# GLIICAN (ESP32)

CAN library for GL-II KV40 Gimbal motor from CubeMars written for ESP32

Install this GLIICAN folder in your Arduino sketchbook's libraries directory, then open File > Examples > GLIICAN > Diagnostics. This uses the legacy driver/twai.h API, tested with Arduino ESP32 2.0.16 on CoreS3.

All project choices are in glii::Config: pins, bitrate, command IDs, reply ID/type, single-shot TX, and encoding ranges. Construct GLIICAN from the config, register onFeedback once, call begin once, and call update continuously. One instance owns the global TWAI controller; do not install another TWAI driver or separately drain its receive queue.

The sample retains motor ID 1, 1 Mbit/s, and ranges 12.5/30/10 from your source. Reply ID 1 MUST be verified. Earlier notes said velocity range 200; both transmit and receive scaling must match the actual motor configuration. Velocity units also require confirmation. Position/velocity and velocity command IDs are explicit because mode-specific addressing has not been established; the library does not switch the board's saved control mode.

Commands return whether the CAN frame was queued, not whether the motor executed it. begin() does not enable the motor. The example accepts e/x/z/c serial commands for enable/disable/zero/clear errors and sends zero-gain, zero-torque MIT packets while explicitly enabled. Startup waits for valid feedback but still accepts these serial commands. Feedback presence does not imply a healthy or enabled motor.

Request tracking matches full reply ID, standard/extended type and length. Without a protocol sequence field, unsolicited telemetry or late replies with the same signature can satisfy a pending request. Call from a single task, not an ISR; callbacks receive temporary references and should copy data they retain.