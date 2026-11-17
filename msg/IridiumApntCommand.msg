# Request to the Iridium APNT receiver: read or write a parameter, or run a command
#
# The driver answers with an iridium_apnt_response carrying the same request_id.

uint64 timestamp        # [us] Time since system start

uint32 request_id       # [-] Echoed by the receiver, used to match the response

uint8 type              # [@enum TYPE]
uint8 TYPE_GET     = 0  # $GSGET, read parameter param_id
uint8 TYPE_SET     = 1  # $GSSET, write parameter param_id with args
uint8 TYPE_REQUEST = 2  # $GSREQ, run request param_id with args. 1: reset (arg 4 = reboot), 5: clear one NVM parameter, 6: clear all NVM parameters

uint16 param_id         # [-] Parameter ID, or request ID for TYPE_REQUEST
bool nvm                # true = non-volatile, false = volatile (current session)
uint8[96] args          # Comma separated arguments, null terminated. Empty for none

uint8 ORB_QUEUE_LENGTH = 4
