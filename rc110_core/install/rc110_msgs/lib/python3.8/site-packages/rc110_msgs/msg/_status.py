# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc110_msgs:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MOTOR_OFF': 0,
        'MOTOR_ON': 1,
        'MOTOR_NEUTRAL': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rc110_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rc110_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MOTOR_OFF': cls.__constants['MOTOR_OFF'],
            'MOTOR_ON': cls.__constants['MOTOR_ON'],
            'MOTOR_NEUTRAL': cls.__constants['MOTOR_NEUTRAL'],
        }

    @property
    def MOTOR_OFF(self):
        """Message constant 'MOTOR_OFF'."""
        return Metaclass_Status.__constants['MOTOR_OFF']

    @property
    def MOTOR_ON(self):
        """Message constant 'MOTOR_ON'."""
        return Metaclass_Status.__constants['MOTOR_ON']

    @property
    def MOTOR_NEUTRAL(self):
        """Message constant 'MOTOR_NEUTRAL'."""
        return Metaclass_Status.__constants['MOTOR_NEUTRAL']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      MOTOR_OFF
      MOTOR_ON
      MOTOR_NEUTRAL
    """

    __slots__ = [
        '_board_enabled',
        '_motor_state',
        '_servo_state',
    ]

    _fields_and_field_types = {
        'board_enabled': 'boolean',
        'motor_state': 'uint8',
        'servo_state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.board_enabled = kwargs.get('board_enabled', bool())
        self.motor_state = kwargs.get('motor_state', int())
        self.servo_state = kwargs.get('servo_state', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.board_enabled != other.board_enabled:
            return False
        if self.motor_state != other.motor_state:
            return False
        if self.servo_state != other.servo_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def board_enabled(self):
        """Message field 'board_enabled'."""
        return self._board_enabled

    @board_enabled.setter
    def board_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'board_enabled' field must be of type 'bool'"
        self._board_enabled = value

    @property
    def motor_state(self):
        """Message field 'motor_state'."""
        return self._motor_state

    @motor_state.setter
    def motor_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor_state' field must be an unsigned integer in [0, 255]"
        self._motor_state = value

    @property
    def servo_state(self):
        """Message field 'servo_state'."""
        return self._servo_state

    @servo_state.setter
    def servo_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'servo_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'servo_state' field must be an unsigned integer in [0, 255]"
        self._servo_state = value
