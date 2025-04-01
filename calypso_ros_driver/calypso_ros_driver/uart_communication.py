import serial
import threading

class UART_communication:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.callback = None
        self.thread = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.thread = threading.Thread(target=self._read_data)
        self.thread.daemon = True
        self.thread.start()

    def _read_data(self):
        while self.ser and self.ser.is_open:
            data = self.ser.readline()
            if data and self.callback:
                self.callback(data.decode('utf-8').strip())

    def set_callback(self, callback):
        self.callback = callback

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

