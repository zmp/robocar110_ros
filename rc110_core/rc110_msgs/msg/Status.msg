#
# Drive or servo motor states:
#
uint8 MOTOR_OFF = 0      # No current
uint8 MOTOR_ON = 1       # On and not movable by hand
uint8 MOTOR_NEUTRAL = 2  # On but movable by hand

#
# Status Message
#
bool board_enabled
uint8 motor_state
uint8 servo_state
