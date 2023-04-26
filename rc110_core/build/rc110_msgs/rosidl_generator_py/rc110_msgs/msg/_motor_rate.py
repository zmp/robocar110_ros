# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc110_msgs:msg/MotorRate.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorRate(type):
    """Metaclass of message 'MotorRate'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'rc110_msgs.msg.MotorRate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_rate
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_rate
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_rate
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_rate
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_rate

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorRate(metaclass=Metaclass_MotorRate):
    """Message class 'MotorRate'."""

    __slots__ = [
        '_header',
        '_motor_rate',
        '_estimated_speed',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_rate': 'float',
        'estimated_speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_rate = kwargs.get('motor_rate', float())
        self.estimated_speed = kwargs.get('estimated_speed', float())

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
        if self.header != other.header:
            return False
        if self.motor_rate != other.motor_rate:
            return False
        if self.estimated_speed != other.estimated_speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def motor_rate(self):
        """Message field 'motor_rate'."""
        return self._motor_rate

    @motor_rate.setter
    def motor_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor_rate' field must be of type 'float'"
        self._motor_rate = value

    @property
    def estimated_speed(self):
        """Message field 'estimated_speed'."""
        return self._estimated_speed

    @estimated_speed.setter
    def estimated_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'estimated_speed' field must be of type 'float'"
        self._estimated_speed = value
