# Iridium APNT burst statistics for the prior minute ($GSSTA)
#
# Unknown floats are NaN, unknown counts are -1.

uint64 timestamp        # [us] Time since system start, at reception of the sentence
uint32 device_id        # [-] Unique device ID for the receiver

int64 bursts_total      # [-] Running total of processed bursts
int32 bursts_last_minute # [-] Bursts processed in the prior minute
float32 cn0_avg         # [dB-Hz] Average burst C/N0
float32 cn0_max         # [dB-Hz]
float32 cn0_min         # [dB-Hz]
