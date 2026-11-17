# Iridium APNT velocity solution ($GSVEL)
#
# Unknown floats are NaN, unknown coded values are -1.

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

uint64 gps_time_us      # [us] Time of validity, since the GPS epoch (1980-01-06), 0 if unknown

float32[3] velocity_enu # [m/s] East, North, Up velocity
float32[9] covariance   # [m^2/s^2] Velocity covariance, row-major, East North Up

float32 burst_age       # [s] Time since the last Iridium burst
float32 sensor_age      # [s] Time since the last sensor update
float32 kf_age          # [s] Time since the last Kalman filter update

int8 state              # [@enum STATE] Validation state of the PVT solution
int8 STATE_UNKNOWN       = -1
int8 STATE_NOT_AVAILABLE = 0
int8 STATE_INITIALIZING  = 1
int8 STATE_PENDING       = 2
int8 STATE_USABLE        = 3
