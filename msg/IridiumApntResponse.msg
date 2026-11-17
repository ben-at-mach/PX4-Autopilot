# Result of an iridium_apnt_command ($GSRSP, with the $GSPAR values for a read)

uint64 timestamp        # [us] Time since system start

uint32 request_id       # [-] From the command

bool ack                # The receiver accepted the command
bool timeout            # No answer within the driver timeout, error_code is not meaningful
int8 error_code         # [@enum ERROR] Receiver error code from $GSRSP
int8 ERROR_NONE                          = 0
int8 ERROR_INVALID_ARG_NB                = -1
int8 ERROR_UNSUPPORTED_PARAM_ID          = -2
int8 ERROR_UNSUPPORTED_MEMORY_CLASS      = -3
int8 ERROR_UNSUPPORTED_REQUEST_ID        = -5
int8 ERROR_PARAMETER_VALUE_TOO_LONG      = -6
int8 ERROR_PARAMETER_IS_UNSET            = -7
int8 ERROR_INVALID_ARG                   = -8
int8 ERROR_CANNOT_CHANGE_OTP_PARAMETER   = -9
int8 ERROR_INVALID_OPERATION             = -10
int8 ERROR_UNSUPPORTED_SENTENCE_VERSION  = -11

uint16 param_id         # [-] Parameter ID reported by $GSPAR for a read
uint8[96] args          # Comma separated values from $GSPAR for a read, null terminated

uint8 ORB_QUEUE_LENGTH = 4
