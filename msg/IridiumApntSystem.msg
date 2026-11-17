# Iridium APNT system status ($GSSYS)

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

uint64 uptime_s         # [s] System uptime
uint64 app_uptime_s     # [s] Main application uptime

uint32 error            # [@enum ERROR] Bitmask of receiver errors
uint32 ERROR_MISSING_SERIAL_NUMBER = 1
uint32 ERROR_MISSING_SUBSCRIPTION  = 2
uint32 ERROR_MISSING_MODEL         = 4
uint32 ERROR_MISSING_MANUFACTURER  = 8
uint32 ERROR_MISSING_KEY           = 16
uint32 ERROR_INVALID_BTFC          = 32 # Broadcast time frame count not resolved yet
uint32 ERROR_OPERATIONAL_LIMITS    = 64 # Set from initialization until the receiver converges within operating limits

uint32 subscription_configured # [@enum SUBSCRIPTION] Bitmask
uint32 subscription_enabled    # [@enum SUBSCRIPTION] Bitmask
uint32 SUBSCRIPTION_LOCATION = 1
uint32 SUBSCRIPTION_TIMING   = 2
uint32 SUBSCRIPTION_DYNAMIC  = 16
uint32 SUBSCRIPTION_RAW      = 32

int64 subscription_start_gps_s # [s] Since the GPS epoch, -1 if unknown
int64 subscription_end_gps_s   # [s] Since the GPS epoch, -1 if unknown
uint32 extended_status         # [-] Reserved bits

uint8[24] version # Receiver application software version, null terminated
