# Iridium APNT position solution ($GSPOS)
#
# Unknown floats are NaN, unknown coded values are -1.

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

uint64 gps_time_us      # [us] Time of validity, since the GPS epoch (1980-01-06), 0 if unknown

float64 latitude           # [deg] Latitude
float64 longitude          # [deg] Longitude
float64 altitude_ellipsoid # [m] Altitude above the WGS84 ellipsoid
float64[3] ecef            # [m] ECEF X, Y, Z
float32[9] covariance      # [m^2] Position covariance, row-major, East North Up

float32 burst_age       # [s] Time since the last Iridium burst
float32 kf_age          # [s] Time since the last Kalman filter update

int8 pfom               # [@enum PFOM] Position figure of merit, 1 (best, <25 m RMS) to 9 (>=5 km). Values above 9 have been seen and mean worse than 9
int8 mode               # [@enum MODE] Position mode
int8 MODE_UNKNOWN       = -1
int8 MODE_STATIC_KNOWN  = 0 # Static, surveyed position
int8 MODE_STATIC_UNKNOWN = 1 # Static, guessed position
int8 MODE_DYNAMIC       = 3
