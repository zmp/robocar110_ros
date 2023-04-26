#
# All errors that can be emitted from baseboard:
#
uint8 NONE = 0
uint8 BOARD_HEAT = 1     # board has too high temperature
uint8 MOTOR_HEAT = 2     # motor has too high temperature
uint8 MOTOR_FAILURE = 3  # encoder feedback and polarity of the motor
uint8 LOW_VOLTAGE = 4    # voltage dropped less than 6V for around 1s

#
# Error
#
uint8 data
