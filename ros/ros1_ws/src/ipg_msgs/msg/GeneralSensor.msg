# Sensor State Constants (definitions)
uint8 VALID = 0
uint8 FUSE_BLOWN = 1
uint8 UPPER_LIMIT_EXCEEDED = 2
uint8 LOWER_LIMIT_EXCEEDED = 3
uint8 Timeout = 4

Header header
float32 SensorValue
uint8 State