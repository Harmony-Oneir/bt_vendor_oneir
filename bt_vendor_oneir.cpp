/*
 * libbt_vendor.z.so for the oneir bench (x86_64, OpenHarmony 7.0).
 *
 * The controller (QCA9377, 0cf3:e500) is driven by the kernel btusb driver,
 * which also loads the firmware during hci0 setup. This library gives the
 * OHOS HCI HDI driver raw access to hci0 through a HCI_CHANNEL_USER socket.
 *
 * The HDI driver reads its channel as an H4 byte stream (type, header, payload
 * in separate reads), while a HCI_CHANNEL_USER socket is packet based: one
 * read() returns one whole packet and drops anything that does not fit.
 * So the driver gets one end of an AF_UNIX stream socketpair, and a bridge
 * thread moves packets between that stream and the user channel socket.
 */

#include <cstddef>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <atomic>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "hilog/log.h"
#include "ohos_bt_vendor_lib.h"

#undef LOG_DOMAIN
#undef LOG_TAG
#define LOG_DOMAIN 0xD000105
#define LOG_TAG "bt_vendor_oneir"
#define LOGI(...) HILOG_INFO(LOG_CORE, __VA_ARGS__)
#define LOGE(...) HILOG_ERROR(LOG_CORE, __VA_ARGS__)

namespace {
// Linux Bluetooth socket constants (the musl sysroot has no <bluetooth/*.h>).
constexpr int AF_BLUETOOTH_ = 31;
constexpr int BTPROTO_HCI_ = 1;
constexpr unsigned short HCI_CHANNEL_USER_ = 1;
constexpr unsigned long HCIDEVDOWN_ = _IOW('H', 202, int);

constexpr unsigned short HCI_DEV = 0;          // hci0
constexpr uint32_t LPM_TIMER_MS = 3000;        // LPM ops are no-ops; avoid a 0 ms watcher timeout
constexpr int BIND_ATTEMPTS = 50;              // 50 x 100 ms: wait for btusb setup / hci0 to appear
constexpr useconds_t BIND_RETRY_US = 100000;

// H4 packet indicators
constexpr uint8_t H4_CMD = 0x01;
constexpr uint8_t H4_ACL = 0x02;
constexpr uint8_t H4_SCO = 0x03;
constexpr uint8_t H4_ISO = 0x05;

struct SockaddrHci {
    sa_family_t family;
    unsigned short dev;
    unsigned short channel;
};

struct RfkillEvent {        // struct rfkill_event v1 (8 bytes)
    uint32_t idx;
    uint8_t type;
    uint8_t op;
    uint8_t soft;
    uint8_t hard;
} __attribute__((packed));
constexpr uint8_t RFKILL_TYPE_BLUETOOTH_ = 2;
constexpr uint8_t RFKILL_OP_CHANGE_ALL_ = 3;

const BtVendorCallbacksT *g_callbacks = nullptr;
int g_userSock = -1;
int g_pair[2] = {-1, -1};      // [0] -> HDI driver, [1] -> bridge
int g_stopPipe[2] = {-1, -1};
std::thread g_bridge;
std::atomic<bool> g_running{false};

void CloseFd(int &fd)
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

void RfkillUnblock()
{
    int fd = open("/dev/rfkill", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        LOGI("rfkill: open /dev/rfkill: %{public}s (skipped)", strerror(errno));
        return;
    }
    RfkillEvent ev = {0, RFKILL_TYPE_BLUETOOTH_, RFKILL_OP_CHANGE_ALL_, 0, 0};
    if (write(fd, &ev, sizeof(ev)) != static_cast<ssize_t>(sizeof(ev))) {
        LOGE("rfkill: unblock bluetooth: %{public}s", strerror(errno));
    }
    close(fd);
}

// The kernel may have brought hci0 up; a user channel can only bind to a down device.
void HciDeviceDown()
{
    int s = socket(AF_BLUETOOTH_, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI_);
    if (s < 0) {
        LOGE("hci control socket: %{public}s", strerror(errno));
        return;
    }
    if (ioctl(s, HCIDEVDOWN_, HCI_DEV) < 0 && errno != EALREADY) {
        LOGE("HCIDEVDOWN hci%{public}d: %{public}s", HCI_DEV, strerror(errno));
    }
    close(s);
}

int OpenUserChannel()
{
    for (int attempt = 0; attempt < BIND_ATTEMPTS; ++attempt) {
        HciDeviceDown();
        int s = socket(AF_BLUETOOTH_, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI_);
        if (s < 0) {
            LOGE("user channel socket: %{public}s", strerror(errno));
            return -1;
        }
        SockaddrHci addr = {};
        addr.family = AF_BLUETOOTH_;
        addr.dev = HCI_DEV;
        addr.channel = HCI_CHANNEL_USER_;
        if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
            LOGI("hci%{public}d bound as HCI_CHANNEL_USER (attempt %{public}d)", HCI_DEV, attempt);
            return s;
        }
        int err = errno;
        close(s);
        LOGE("bind hci%{public}d user channel, attempt %{public}d: %{public}s", HCI_DEV, attempt, strerror(err));
        if (err != EBUSY && err != EALREADY && err != ENODEV) {
            return -1;      // e.g. EPERM: bluetooth_host lacks CAP_NET_ADMIN
        }
        usleep(BIND_RETRY_US);
    }
    return -1;
}

bool WriteAll(int fd, const uint8_t *data, size_t len)
{
    while (len > 0) {
        ssize_t n = write(fd, data, len);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            LOGE("bridge write: %{public}s", strerror(errno));
            return false;
        }
        data += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

// Total H4 packet length (type byte included), 0 if the header is incomplete, SIZE_MAX on a bad type.
size_t H4PacketLen(const std::vector<uint8_t> &a)
{
    if (a.empty()) {
        return 0;
    }
    switch (a[0]) {
        case H4_CMD:    // type, opcode(2), plen(1)
            return a.size() < 4 ? 0 : 4 + a[3];
        case H4_ACL:    // type, handle(2), dlen(2)
            return a.size() < 5 ? 0 : 5 + (a[3] | (a[4] << 8));
        case H4_SCO:    // type, handle(2), dlen(1)
            return a.size() < 4 ? 0 : 4 + a[3];
        case H4_ISO:    // type, handle(2), dlen(14 bits)
            return a.size() < 5 ? 0 : 5 + ((a[3] | (a[4] << 8)) & 0x3fff);
        default:
            return SIZE_MAX;
    }
}

// Host -> controller: cut the H4 stream into packets, one write() per packet.
bool FlushToController(std::vector<uint8_t> &acc)
{
    for (;;) {
        size_t len = H4PacketLen(acc);
        if (len == SIZE_MAX) {
            LOGE("bad H4 packet type 0x%{public}02x from host, dropping %{public}zu bytes", acc[0], acc.size());
            acc.clear();
            return true;
        }
        if (len == 0 || acc.size() < len) {
            return true;
        }
        ssize_t n = write(g_userSock, acc.data(), len);
        if (n != static_cast<ssize_t>(len)) {
            LOGE("write to hci%{public}d: %{public}s", HCI_DEV, n < 0 ? strerror(errno) : "short write");
            return false;
        }
        acc.erase(acc.begin(), acc.begin() + static_cast<std::ptrdiff_t>(len));
    }
}

void Bridge()
{
    std::vector<uint8_t> fromController(65536);
    std::vector<uint8_t> fromHost;
    fromHost.reserve(65536);
    uint8_t buf[4096];

    pollfd fds[3] = {
        {g_userSock, POLLIN, 0},
        {g_pair[1], POLLIN, 0},
        {g_stopPipe[0], POLLIN, 0},
    };
    LOGI("bridge started");
    while (g_running.load()) {
        int r = poll(fds, 3, -1);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            LOGE("bridge poll: %{public}s", strerror(errno));
            break;
        }
        if (fds[2].revents != 0) {
            break;
        }
        if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            LOGE("hci%{public}d user channel closed by kernel", HCI_DEV);
            break;
        }
        if (fds[0].revents & POLLIN) {
            // One read() == one packet, already prefixed with its H4 type byte.
            ssize_t n = read(g_userSock, fromController.data(), fromController.size());
            if (n > 0) {
                if (!WriteAll(g_pair[1], fromController.data(), static_cast<size_t>(n))) {
                    break;
                }
            } else if (n < 0 && errno != EINTR && errno != EAGAIN) {
                LOGE("read hci%{public}d: %{public}s", HCI_DEV, strerror(errno));
                break;
            }
        }
        if (fds[1].revents & (POLLIN | POLLHUP)) {
            ssize_t n = read(g_pair[1], buf, sizeof(buf));
            if (n == 0) {
                LOGI("host side of the channel closed");
                break;
            }
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }
                LOGE("read host stream: %{public}s", strerror(errno));
                break;
            }
            fromHost.insert(fromHost.end(), buf, buf + n);
            if (!FlushToController(fromHost)) {
                break;
            }
        }
    }
    LOGI("bridge stopped");
}

void ChannelClose()
{
    if (g_bridge.joinable()) {
        g_running.store(false);
        if (g_stopPipe[1] >= 0) {
            (void)write(g_stopPipe[1], "x", 1);
        }
        g_bridge.join();
    }
    g_running.store(false);
    CloseFd(g_stopPipe[0]);
    CloseFd(g_stopPipe[1]);
    CloseFd(g_pair[0]);
    CloseFd(g_pair[1]);
    CloseFd(g_userSock);    // releases hci0 back to the kernel
}

int ChannelOpen(int *channels)
{
    if (channels == nullptr) {
        return -1;
    }
    ChannelClose();
    g_userSock = OpenUserChannel();
    if (g_userSock < 0) {
        return -1;
    }
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, g_pair) < 0 || pipe2(g_stopPipe, O_CLOEXEC) < 0) {
        LOGE("socketpair/pipe: %{public}s", strerror(errno));
        ChannelClose();
        return -1;
    }
    g_running.store(true);
    g_bridge = std::thread(Bridge);
    channels[0] = g_pair[0];
    LOGI("HCI channel open: H4 over fd %{public}d", g_pair[0]);
    return 1;   // one channel -> the HDI driver uses H4Protocol
}

int VendorInit(const BtVendorCallbacksT *callbacks, unsigned char *localBdaddr)
{
    (void)localBdaddr;  // the controller keeps its own address
    g_callbacks = callbacks;
    LOGI("init (hci%{public}d)", HCI_DEV);
    return 0;
}

int VendorOp(BtOpcodeT opcode, void *param)
{
    switch (opcode) {
        case BT_OP_POWER_ON:
            LOGI("power on");
            RfkillUnblock();
            return 0;
        case BT_OP_POWER_OFF:
            LOGI("power off");
            return 0;
        case BT_OP_HCI_CHANNEL_OPEN:
            return ChannelOpen(static_cast<int *>(param));
        case BT_OP_HCI_CHANNEL_CLOSE:
            LOGI("channel close");
            ChannelClose();
            return 0;
        case BT_OP_INIT:
            // Firmware is already in the controller (btusb did it), nothing to configure.
            // Report completion asynchronously, like a real vendor library would.
            LOGI("controller init: nothing to do, reporting success");
            std::thread([] {
                if (g_callbacks != nullptr && g_callbacks->initCb != nullptr) {
                    g_callbacks->initCb(BTC_OP_RESULT_SUCCESS);
                }
            }).detach();
            return 0;
        case BT_OP_GET_LPM_TIMER:
            if (param != nullptr) {
                *static_cast<uint32_t *>(param) = LPM_TIMER_MS;
            }
            return 0;
        case BT_OP_LPM_ENABLE:
        case BT_OP_LPM_DISABLE:
        case BT_OP_WAKEUP_LOCK:
        case BT_OP_WAKEUP_UNLOCK:
        case BT_OP_EVENT_CALLBACK:  // only used for commands sent via xmitCb; we send none
        default:
            return 0;
    }
}

void VendorClose()
{
    LOGI("close");
    ChannelClose();
    g_callbacks = nullptr;
}
}  // namespace

extern "C" __attribute__((visibility("default"))) BtVendorInterfaceT BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(BtVendorInterfaceT),
    VendorInit,
    VendorOp,
    VendorClose,
};
