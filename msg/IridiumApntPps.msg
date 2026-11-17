# Iridium APNT timing status ($GSPPS)
#
# Unknown floats are NaN, unknown coded values are -1.

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

int64 gps_time_s        # [s] GPS time of the last PPS rising edge, -1 if unknown
int32 utc_offset_s      # [s] GPS minus UTC, -1 if invalid
int8 leap               # [-] Pending leap second

int8 pps_status         # [@enum PPS_STATUS] PPS output state
int8 PPS_STATUS_INIT              = 0
int8 PPS_STATUS_FREERUN           = 1
int8 PPS_STATUS_HOLDOVER          = 2
int8 PPS_STATUS_TRACKING_ACTIVE   = 3
int8 PPS_STATUS_TRACKING_LOCKABLE = 4

int8 source             # [-] PPS source: 0 none, 1 Iridium PNT
float32 accuracy_ns     # [ns] PPS accuracy, 1 sigma
int8 tfom               # [-] Time figure of merit, 1 (<=1 ns) to 15 (>10 s), one step per decade

bool hard_correction_coming  # A hard correction (slew slam) is about to happen
bool hard_correction_applied # A hard correction was just applied

int8 iridium_pps_status # [-] Iridium PPS state: 0 init, 1 acquiring, 2 lockable, 3 active
float32 burst_age       # [s] Time since the last Iridium burst, -1 if none yet
float32 iridium_diff_ns # [ns] PPS out minus Iridium PPS, positive when PPS out is late
float32 iridium_accuracy_ns # [ns] Iridium PPS accuracy
