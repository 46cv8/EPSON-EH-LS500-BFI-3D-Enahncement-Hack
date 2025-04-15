# EPSON-EH-LS500-BFI-3D-Enahncement-Hack
Firmware and some pictures for an attempt at improving 3D performance on the Epson EH-LS500 3LCD projector by using optimally timed pulsing of the laser light source
I posted most of my analysis and thought process for developing this hack at https://www.avforums.com/threads/epson-ls500-and-3d-crosstalk.2463787/

# End Result
Unfortunately in final testing I realized that the digital gain applied by my high speed machine vision camera had been crushing black levels in my 2139 hz test footage.
I had been under the impression that the 3 panels in the projector had a full pixel response time of around 3-4 ms, as the footage I had collected showed this to the the case.
What I found was that the cameras digital gain had crushed the dyamic range so that it wasn' showing there was still nearly 10% ghosting  even after the initial 3-4 ms of pixel response time.

# Implementation
It uses a single ESP32 WROOM32 dev board with an additional buck converter board to take the 19.2v line down to 5v for the board.
The board intercepts the 3d sync signal that goes to the 3d bluetooth module in tep projector via one harness
The board also performs a MITM (main in the middle) on the PWM1, TXD and RXD lines between the main projector microcontroller and the laser driver board (more details are on the thread at avforums.
The firmware supports OTA bluetooth updates, and changing parameters via bluetooth using a companion script.
It also supports dumping the TXD RXD serial logs between the main MCU and laser driver board for changing it to potentially work with different Epson projectors.
The first mode of operation is PWM MITM which gates the PWM1 signal from the main MCU) with the timing parameters for frame delay and frame duration you specify, based on the timing signal from the 3d sync line.
THe second mode of operation is Serial MITM which intercepts all serial coms and tricks the main MCU to think it is in control of the laser brightness all whilst overriding it with brightness values you specify.
This lets you get a brighter image without needing to hear exessive fan noise. (beause you are PWM gating the laser diode by more than 50% it is already akin to running in ECO mode so you shouldn't need full fan speed for full brightness).

# Artifacts
- ESP32 c firmware
- Python bluetooth control script
- Gimp image file of laser driver board analysis
- Datasheets for various chis used on the laser driver board (shows the pinout for the connector)

# Questions
If you have any questions about making this yourself feel free to create an issue and I'll try to help if I can.
