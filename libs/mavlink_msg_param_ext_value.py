# More information about this file can be found at: mavlink_msg_param_ext_value.h 

# typedef struct __mavlink_param_ext_value_t {
#  uint16_t param_count; /*<  Total number of parameters*/
#  uint16_t param_index; /*<  Index of this parameter*/
#  char param_id[16]; /*<  Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
#  char param_value[128]; /*<  Parameter value*/
#  uint8_t param_type; /*<  Parameter type.*/
# } mavlink_param_ext_value_t;

MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN = 149

# MAVLINK_MSG_ID_PARAM_EXT_VALUE_LEN = 149 byte (2 byte param_count + 2 byte param_index + 16 byte param_id + 128 byte param_value + 1 byte param_type).