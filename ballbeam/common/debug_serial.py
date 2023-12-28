from serial import Serial

from ballbeam.configurators.configs import CONFIG

# TODO figure out why servo sometimes jumps when starting (or stopping?) the system
# Idea: might be when Serial() or ser.close() is called it sends out some data that gets read by the Arduino and converted to a valid servo command pwm
# Fix: make a state machine in the Arduino control.ino sketch and use a sentinel value e.g. 0 to tell the Arduino to always write servo commmand pwm to rest

ser = Serial(CONFIG.hardware.COMM.PORT, CONFIG.hardware.COMM.BAUD_RATE, timeout=1)
# ser.close()
