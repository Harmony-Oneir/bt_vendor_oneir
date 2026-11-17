# bt_vendor_oneir

Vendor-библиотека Bluetooth (`libbt_vendor.z.so`) для **oneir** — OpenHarmony 7.0 на x86_64.

HCI HDI-драйвер OpenHarmony (`drivers/peripheral/bluetooth/hci`) загружает её, чтобы получить
транспорт до контроллера. На стенде USB-контроллер (QCA9377, `0cf3:e500`) обслуживает драйвер
ядра `btusb`, он же загружает прошивку. Библиотека открывает `hci0` как сокет `HCI_CHANNEL_USER`
и отдаёт его HDI-драйверу в режиме H4.

## Как устроено

    bluetooth_service → HDI hci_interface_service (bluetooth_host)
                      → libbt_vendor.z.so → HCI_CHANNEL_USER → hci0 (btusb) → контроллер

- `BT_OP_HCI_CHANNEL_OPEN` — опускает `hci0`, делает `bind` в `HCI_CHANNEL_USER`, возвращает один канал (H4).
- Сокет user channel пакетный, а драйвер читает H4 как поток, поэтому между ними стоит мост
  (`socketpair` + поток), который режет поток на целые HCI-пакеты.
- `BT_OP_INIT` — сразу сообщает об успехе: прошивку уже загрузил `btusb`.
- LPM и wakeup — пустые операции.

## Требования

- Ядро: `CONFIG_BT`, `CONFIG_BT_HCIBTUSB`; прошивка контроллера в `/lib/firmware`.
- `bluetooth_host` в HCS с `caps = ["NET_ADMIN", "NET_RAW"]` (без `NET_ADMIN` `bind` вернёт `EPERM`).
- `hci0` не должен быть занят другими программами (bluetoothd, `hciconfig up`).

## Сборка

Клонирование репозитория:
```bash
git clone https://github.com/Harmony-Oneir/bt_vendor_oneir.git ./oneir/bluetooth/vendor
```

Каталог кладётся в `oneir/bluetooth/vendor/`, затем:
```bash
    ./make.sh oneir/bluetooth/vendor:bt_vendor
```
В образе: `/vendor/lib64/libbt_vendor.z.so`.

## Отладка
```bash
hilog -x | grep bt_vendor_oneir
```
