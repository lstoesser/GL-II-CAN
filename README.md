# GLIICAN (ESP32)

CAN library for GL-II KV40 Gimbal motor from CubeMars written for ESP32

The motor must first be configured in the CubeMars upper computer software GUI. Connect the motor's serial port, CAN port, and power interface with the S-link interface. On the computer, open the upper computer software and select the corresponding serial port device to open the serial port. 

Click Calibrate, the motor will rotate first, and after the rotation is completed, it will return the number of pole pairs to the GUI.

Confirm that the motor input power is stable, the S-link connection is normal, and after successfully connecting with the upper computer, click on "Set parameters" and "Read Parameters" in sequence, then click on "Control Mode" in the control settings to select MIT mode, speed-position mode, or velocity mode, and then click "Write Parameters" to set the control mode. Most of the firmware we will be using requires the motor to be in MIT Mode.

Install this GLIICAN folder in your Arduino sketchbook's libraries directory, then open File > Examples > GLIICAN > MITSetUpGuideArduinoExample. This mimics the set up guide provided by cube mars. https://www.cubemars.com/data/cms/202604/gl-ii-gimbal-motor-drive-user-manual.pdf Ensure that the motor is in MIT mode and the CAN ID and baud rate match the firmware.

Commands return whether the CAN frame was queued, not whether the motor executed it. begin() does not enable the motor. The example accepts e/x/z/c serial commands for enable/disable/zero/clear errors and sends zero-gain, zero-torque MIT packets while explicitly enabled. Startup waits for valid feedback but still accepts these serial commands. Feedback presence does not imply a healthy or enabled motor.

Request tracking matches full reply ID. Without a protocol sequence field, unsolicited telemetry or late replies with the same signature can satisfy a pending request. Call from a single task, not an ISR; callbacks receive temporary references and should copy data they retain.
