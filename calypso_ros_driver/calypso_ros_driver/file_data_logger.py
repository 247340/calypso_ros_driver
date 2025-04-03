import time
import datetime
from zoneinfo import ZoneInfo 
import os
#Initializes the logger with a file name and flush interval.
class Data_logger:
    def __init__(self, filename='calypso_log.csv', flush_interval=5):
        self.base_folder = os.path.join(os.getcwd(), 'src/calypso_ros_driver') 
        self.log_folder = os.path.join(self.base_folder, 'log') 
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
        self.filename = os.path.join(self.log_folder, filename)
        self.flush_interval = flush_interval # interval in seconds
        self.buffer = [] # buffer for storing data
        self.last_flush_time = time.time()  # last flush time
        with open(self.filename, 'w') as file:
                    file.write("DATETIME,TIMESTAMP,DIRECTION,SPEED\n")
#Adds data to the buffer and writes to the file if flush interval has passed.
    def write_data_to_file(self, timestamp, direction, speed):
        timestamp_sec = timestamp / 1_000_000
        dt_object = datetime.datetime.utcfromtimestamp(timestamp_sec)
        local_time = dt_object.replace(tzinfo=ZoneInfo("UTC")).astimezone(ZoneInfo("Europe/Prague"))
        formatted_time = local_time.strftime('%Y-%m-%d %H:%M:%S') + f".{local_time.microsecond // 1000:03d}"
        data = f"{formatted_time},{timestamp},{direction},{speed}"
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

