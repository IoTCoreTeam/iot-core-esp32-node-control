# iot-core-esp32-node-control

ESP32 control node for `node-control-001` (pump/light).

## Environments

Office WiFi:
- `control-001`

Home WiFi:
- `control-001-home`

## Runtime Behavior

- Receives `relay_control` command from gateway via ESP-NOW.
- Controls relays:
  - `GPIO23` -> pump
  - `GPIO22` -> light
- Sends node heartbeat every 5 seconds.
- Sends an immediate controller status event after every command is handled.
- The status event includes command result metadata in `status_kv`:
  - `cmd`: command sequence from gateway
  - `cd`: device (`pump` / `light`)
  - `ct`: requested target state
  - `cr`: execution result (`applied`, `invalid_state`, `unknown_device`)
- WiFi channel is resolved automatically from configured SSID.

## MAC Configuration

- Gateway MAC is defined by `GATEWAY_MAC_*` in `src/main.cpp`.
- Update these bytes if GW_001 MAC changes.

## Build / Upload / Monitor

```bash
pio run -e control-001
pio run -e control-001 -t upload
pio device monitor -e control-001
```

Home example:

```bash
pio run -e control-001-home
pio run -e control-001-home -t upload
pio device monitor -e control-001-home
```
