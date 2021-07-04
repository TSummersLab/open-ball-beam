from time import time, sleep
import struct
from serial import Serial

from settings import DT, SERVO_CMD_MID, SERVO_CMD_REST, SENSOR_MIDPOINT


def create_serial(port=None, baud_rate=None, timeout=1):
    if port is None:
        from settings import PORT
        port = PORT
    if baud_rate is None:
        from settings import BAUD_RATE
        baud_rate = BAUD_RATE
    return Serial(port, baud_rate, timeout=timeout)


class Hardware:
    def __init__(self, ser=None, num_init_reads=3):
        if ser is None:
            self.ser = create_serial()

        # Check to make sure the hardware is working properly
        for i in range(num_init_reads):
            raw_line = self.ser.readline()
            line = raw_line.decode('utf-8').rstrip()
            if 'Failed' in line:
                raise RuntimeError(line)

    def process(self, action, actuation):
        # Send the actuation command to serial
        out = struct.pack('h', actuation)
        self.ser.write(out)
        return None

    def read_int(self):
        raw_line = self.ser.readline()
        line = raw_line.decode('utf-8').rstrip()
        try:
            val = int(line)
        except:
            val = 0
        return val

    def reading2observation(self, reading):
        # arg: reading, in millimeters
        # return: observation, in meters
        observation = 0.001*reading - SENSOR_MIDPOINT
        return observation

    def observe(self):
        # Read the measurement value from serial
        reading = self.read_int()
        # Convert raw reading to a standard observation
        observation = self.reading2observation(reading)
        return observation

    def reset(self, x=None):
        print('resetting system...', end='')
        time_start = time()
        # give time for ball to roll down
        rest_time_seconds = 4.0
        rest_steps = int(rest_time_seconds/DT)
        for i in range(rest_steps):
            out = struct.pack('h', SERVO_CMD_REST)
            self.ser.write(out)
            self.observe()
        time_end = time()
        time_elapsed = time_end - time_start
        print('system reset after resting %.3f seconds' % time_elapsed)
        return

    def shutdown(self):
        self.reset()
        print('shutting down')
        sleep(0.1)
        # Close serial connection
        if self.ser is not None:
            self.ser.close()
        sleep(0.1)
        return
