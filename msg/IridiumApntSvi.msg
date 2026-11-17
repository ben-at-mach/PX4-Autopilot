# Iridium APNT per-burst satellite ranging data ($GSSVI)
#
# sv_id outside 1..66 means the receiver could not identify the satellite; the burst is logged
# only and is not used in the solution. Unknown floats are NaN, unknown coded values are -1.

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

int32 sv_id             # [-] Satellite ID, valid 1..66

uint64 time_of_validity_us # [us] Since the GPS epoch, 0 if unknown
float64 time_of_arrival_s  # [s] Receiver time since initialization
int64 burst_counter     # [-] Increases monotonically

float64 pseudorange     # [m]
float32 pseudorange_rate # [m/s]
float32 cn0             # [dB-Hz]
float32 doppler         # [Hz]
float64[3] sv_position_ecef # [m]
float32[3] sv_velocity_ecef # [m/s]

int8 state              # [-] Validation state: 0 n/a, 1 initializing, 2 pending, 3 usable, -1 unknown

uint8 ORB_QUEUE_LENGTH = 8
