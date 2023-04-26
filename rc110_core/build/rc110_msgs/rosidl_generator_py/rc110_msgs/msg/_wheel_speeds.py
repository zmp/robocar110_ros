# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc110_msgs:msg/WheelSpeeds.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WheelSpeeds(type):
    """Metaclass of message 'WheelSpeeds'."""

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
                'rc110_msgs.msg.WheelSpeeds')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__wheel_speeds
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__wheel_speeds
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__wheel_speeds
            cls._TYPE_SUPPORT = module.type_support_msg__msg__wheel_speeds
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__wheel_speeds

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


class WheelSpeeds(metaclass=Metaclass_WheelSpeeds):
    """Message class 'WheelSpeeds'."""

    __slots__ = [
        '_header',
        '_speed_fl',
        '_speed_fr',
        '_speed_rl',
        '_speed_rr',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'speed_fl': 'float',
        'speed_fr': 'float',
        'speed_rl': 'float',
        'speed_rr': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.speed_fl = kwargs.get('speed_fl', float())
        self.speed_fr = kwargs.get('speed_fr', float())
        self.speed_rl = kwargs.get('speed_rl', float())
        self.speed_rr = kwargs.get('speed_rr', float())

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
        if self.speed_fl != other.speed_fl:
            return False
        if self.speed_fr != other.speed_fr:
            return False
        if self.speed_rl != other.speed_rl:
            return False
        if self.speed_rr != other.speed_rr:
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
    def speed_fl(self):
        """Message field 'speed_fl'."""
        return self._speed_fl

    @speed_fl.setter
    def speed_fl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_fl' field must be of type 'float'"
        self._speed_fl = value

    @property
    def speed_fr(self):
        """Message field 'speed_fr'."""
        return self._speed_fr

    @speed_fr.setter
    def speed_fr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_fr' field must be of type 'float'"
        self._speed_fr = value

    @property
    def speed_rl(self):
        """Message field 'speed_rl'."""
        return self._speed_rl

    @speed_rl.setter
    def speed_rl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_rl' field must be of type 'float'"
        self._speed_rl = value

    @property
    def speed_rr(self):
        """Message field 'speed_rr'."""
        return self._speed_rr

    @speed_rr.setter
    def speed_rr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_rr' field must be of type 'float'"
        self._speed_rr = value
