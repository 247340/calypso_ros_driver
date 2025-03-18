import serial
class UART_communication:
#Initializes the UART communication with the specified port, baud rate, and timeout.
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
#Reads a line of data from the UART connection and returns it as a string.
    def read_data(self):
        if self.ser and self.ser.is_open:
            data = self.ser.readline()
            return data.decode('utf-8').strip()
        return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

