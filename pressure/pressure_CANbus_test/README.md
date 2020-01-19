# Pressure CAN bus test
This code is designed to run a simple test of the ability of the pressure
boards to send and recieve CAN messages. When wired as described below,
pushing the button connected to one board will cause its LED to change, and
will cause it to send a CAN message to the other board instructing it to
set its LED to the opposite state of the original board's LED.

## Wiring
Pin header P6_PT9 connect to push button. Voltage should be normally 5V.
Pin header P6_PT5 connect to LED through ~100 ohm resistor.
The pin to use on each header is the one connected to R7_PTx.
Connect the CANH and CANL pins of the two pressure boards together.
