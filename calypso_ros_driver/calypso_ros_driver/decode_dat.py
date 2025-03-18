import re
import time

# Decode data and return wind speed and direction as floats.
def decode_data(data): 
    match = re.match(r'\$IIMWV,([^,]+),R,([^,]+),M,A', data)
    if match:
        value1 = match.group(1)
        value2 = match.group(2)
        return float(value1), float(value2) 
    return None

# Return the current time in epoch format in microseconds.
def get_time_microseconds():
    uptime_seconds = int(time.time() * 1_000_000)
    return uptime_seconds
