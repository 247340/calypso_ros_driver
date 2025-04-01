import re
# Decode data and return wind speed and direction as floats.
def decode_data(data): 
    match = re.match(r'\$IIMWV,([^,]+),R,([^,]+),M,A', data)
    if match:
        value1 = match.group(1)
        value2 = match.group(2)
        return int(value1), float(value2) 
    return None, None

