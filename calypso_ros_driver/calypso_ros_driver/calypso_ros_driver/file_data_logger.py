import time
#Initializes the logger with a file name and flush interval.
class Data_logger:
    def __init__(self, filename='uart_data_log.csv', flush_interval=5):
        self.filename = filename
        self.flush_interval = flush_interval # interval in seconds
        self.buffer = [] # buffer for storing data
        self.last_flush_time = time.time()  # last flush time
        with open(self.filename, 'w') as file:
                    file.write("TIMESTAMP,DIRECTION,SPEED\n")
#Adds data to the buffer and writes to the file if flush interval has passed.
    def write_data_to_file(self, timestamp, direction, speed):
        data = f"{timestamp},{direction},{speed}"
        self.buffer.append(data)
        current_time = time.time()
        if current_time - self.last_flush_time >= self.flush_interval:
            self.flush_to_file()
#Writes buffered data to the file and clears the buffer.
    def flush_to_file(self):
        if self.buffer:
            try:
                with open(self.filename, 'a') as file:
                    for data in self.buffer:
                        file.write(f"{data}\n")
            except Exception as e:
                print(f"Error writing to file: {e}")
            self.buffer.clear()
            self.last_flush_time = time.time()

