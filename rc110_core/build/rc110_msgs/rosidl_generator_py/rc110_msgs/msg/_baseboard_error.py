# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc110_msgs:msg/BaseboardError.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BaseboardError(type):
    """Metaclass of message 'BaseboardError'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NONE': 0,
        'BOARD_HEAT': 1,
        'MOTOR_HEAT': 2,
        'MOTOR_FAILURE': 3,
        'LOW_VOLTAGE': 4,
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
                'rc110_msgs.msg.BaseboardError')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__baseboard_error
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__baseboard_error
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__baseboard_error
            cls._TYPE_SUPPORT = module.type_support_msg__msg__baseboard_error
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__baseboard_error

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NONE': cls.__constants['NONE'],
            'BOARD_HEAT': cls.__constants['BOARD_HEAT'],
            'MOTOR_HEAT': cls.__constants['MOTOR_HEAT'],
            'MOTOR_FAILURE': cls.__constants['MOTOR_FAILURE'],
            'LOW_VOLTAGE': cls.__constants['LOW_VOLTAGE'],
        }

    @property
    def NONE(self):
        """Message constant 'NONE'."""
        return Metaclass_BaseboardError.__constants['NONE']

    @property
    def BOARD_HEAT(self):
        """Message constant 'BOARD_HEAT'."""
        return Metaclass_BaseboardError.__constants['BOARD_HEAT']

    @property
    def MOTOR_HEAT(self):
        """Message constant 'MOTOR_HEAT'."""
        return Metaclass_BaseboardError.__constants['MOTOR_HEAT']

    @property
    def MOTOR_FAILURE(self):
        """Message constant 'MOTOR_FAILURE'."""
        return Metaclass_BaseboardError.__constants['MOTOR_FAILURE']

    @property
    def LOW_VOLTAGE(self):
        """Message constant 'LOW_VOLTAGE'."""
        return Metaclass_BaseboardError.__constants['LOW_VOLTAGE']


class BaseboardError(metaclass=Metaclass_BaseboardError):
    """
    Message class 'BaseboardError'.

    Constants:
      NONE
      BOARD_HEAT
      MOTOR_HEAT
      MOTOR_FAILURE
      LOW_VOLTAGE
    """

    __slots__ = [
        '_data',
    ]

    _fields_and_field_types = {
        'data': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.data = kwargs.get('data', int())

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
        if self.data != other.data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'data' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'data' field must be an unsigned integer in [0, 255]"
        self._data = value
