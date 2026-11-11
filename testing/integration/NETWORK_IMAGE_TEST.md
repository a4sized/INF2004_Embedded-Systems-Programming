# Team Wi-Fi and MQTT image check

The older `team_ir_handoff_smp1_usb_cdc_wifi_cyw43.uf2` is an **offline**
build (`NET=none MQTT=0`), despite `WIFI=cyw43` in its filename. It cannot
join a hotspot. The ignored `config/wifi_secrets.h` is only read by the
network-enabled builds; do not paste its password into a serial log or commit
it to Git.

## Isolate hotspot association first

Build with `python3 app_program/subsystem3/build_team_wifi_only.py` and flash
`build_make/team_wifi_connection_test_smp1_usb_cdc_wifi_lwip.uf2`. This
dedicated image initializes only Subsystem 1's shared objects needed by the
Wi-Fi callbacks; it does not start motor, IMU, IR, ultrasonic, Vehicle
Controller or MQTT tasks. Open the USB serial monitor at 115200 and
power-cycle the car. Save lines beginning `wifi:` from boot onward. Look for
`wifi: joining`, then `wifi: link up, ip=...`. If the monitor opens after
those one-time messages, the rebuilt test image prints
`wifi test: init=... err=... link=... ip=... polls=...` once per second while
the serial port is open. `link=2` means the station has an IP address. An
increasing `polls` counter means the radio task is still running. If no
`wifi test:` line appears after several seconds, check that the new test UF2
was flashed and that the USB port is open with DTR asserted.

If the link reports SSID not found, check that the phone hotspot is broadcasting on
2.4 GHz and that the configured SSID matches. If authentication is rejected,
check the password and hotspot security mode. If it joins but waits for IP,
check the hotspot DHCP/client list. This image cannot receive MQTT START, so
it is not a motion test. It does not print other subsystem status lines.

## Check the broker and START path

After Wi-Fi reaches `link up`, build with
`python3 app_program/subsystem3/build_team_network.py` and flash
`build_make/team_ir_handoff_smp1_usb_cdc_wifi_lwip_mqtt.uf2`. This image
uses the configured MQTT broker IP, port, client ID and topics. Look for
`mqtt: connected to ...` and `mqtt: subscribed to ...`. If Wi-Fi links but
MQTT does not, verify that the broker is running on the configured host,
listening on the configured port, and reachable from the same hotspot. The
Vehicle Controller still needs a START command and its other readiness gates
before motion. Barcode junction turns remain disabled.

These are compile-checked profiles. Actual hotspot association, broker
connectivity, and START/motion have not yet been observed with these UF2s.
