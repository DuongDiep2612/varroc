# varroc
Firmware for module connverter
Arduino IDE +++++

# Pin mapping 
GY-33 /-----/ Pro mini 5v 16m
VCC   ----- VCC
CT    ----- A5
DR    ----- A4
GND   ----- GND
S0    ----- GND

Button /-----/ Pro mini
Select ----- 2
Signal ----- 3

# Uart
send 'b' ----- Calibration GY-33
send 'a' ----- GY33 send data
send 'Y' ----- ACK

# Get signal sampling 
Choose select button to switch mode which has two modes is run (led white is on) and get a sample (led Red or Green or Blue is on).
Choose the get the signal button to retrieve a data sampling of the signal for the RGB signal.