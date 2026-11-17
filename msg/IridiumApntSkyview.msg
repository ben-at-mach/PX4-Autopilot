# Iridium APNT satellites in view ($GSSSV group, published once the whole group has arrived)
#
# Arrays are parallel and only the first satellite_count entries are valid.
# Unknown elevation and azimuth are -1, unknown C/N0 is NaN.

uint64 timestamp        # [us] Time since system start, at reception of the last sentence of the group
uint32 device_id        # [-] Unique device ID for the receiver

uint8 MAX_SATELLITES = 24

uint8 satellite_count   # [-] Entries stored, at most MAX_SATELLITES
uint8 satellites_in_view # [-] Count reported by the receiver, may exceed MAX_SATELLITES

int16[24] sv_id         # [-]
int16[24] elevation     # [deg] 0..90
int16[24] azimuth       # [deg] 0..359
float32[24] cn0_min     # [dB-Hz]
float32[24] cn0_max     # [dB-Hz]
