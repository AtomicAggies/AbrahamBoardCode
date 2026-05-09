# Abraham Board Code

Abraham is the SD logging board. It receives framed telemetry over I2C, validates and reassembles fixed 98-byte packets, then writes raw packets to SD while logging operational events to a text log.

## What This Firmware Does

- Listens for framed I2C traffic and queues frames from ISR context.
- Accepts only SD-destination frames (header bit0).
- Validates payload checksum and start/continuation ordering.
- Reassembles exactly 98 bytes, then appends packet to binary log file.
- Writes diagnostic entries to rolling text log.

## End-to-End Receive/Log Flow

```text
Wire.onReceive ISR
   |
   v
Frame queue (size 8)
   |
   v
processI2CFrame:
  - header length check
  - destination SD filter
  - checksum verify
  - packet assembly state machine
   |
   v
on 98 bytes complete -> writeTelemetryPacket() -> current-data.bin
                          +
                          appendLog() -> current-log.txt
```

## Framing and Packet Parameters

| Item | Value |
|---|---|
| Telemetry packet size | 98 bytes |
| I2C max frame size | 32 bytes |
| I2C header size | 2 bytes |
| I2C payload per frame | up to 30 bytes |
| Packet assembly timeout | 1000 ms |
| Frame queue size | 8 |
| SD destination flag | byte0 bit0 |
| Start flag | byte0 bit7 |
| Checksum byte | byte1 (sum of payload bytes) |

## SD File Behavior

At boot:

1. Existing `current-log.txt` and `current-data.bin` are rotated to numbered archive names when possible.
2. Fresh `current-log.txt` and `current-data.bin` are created.
3. Runtime status entries are appended to the log.

During operation:

- Valid packet writes append exactly 98 bytes to `current-data.bin`.
- Invalid conditions (checksum failure, overflow, timeout, short frame, queue overflow) are counted and logged.

## Validation and Failure Handling

| Condition | Action |
|---|---|
| Frame shorter than header | Drop frame and discard partial packet |
| Destination bit0 not set | Ignore frame |
| Checksum mismatch | Increment checksum failure counter and discard partial |
| Continuation without active packet | Drop continuation |
| Buffer overflow risk | Discard partial packet |
| Timeout before 98 bytes | Discard partial packet |
| Queue overflow | Increment dropped frame counter and discard partial packet |

## Related Tooling

- Spencer repo includes `telemetry_packet_viewer.py`, which decodes the same 98-byte packet format written by Abraham to `current-data.bin`.
