# One raw NMEA line from the Iridium APNT receiver, without the line terminator

uint64 timestamp        # [us] Time since system start, at reception
uint32 device_id        # [-] Unique device ID for the receiver

bool checksum_ok        # The XOR checksum matched
uint8 len               # [-] Valid bytes in data
uint8[200] data         # Sentence text, truncated if longer

uint8 ORB_QUEUE_LENGTH = 8
