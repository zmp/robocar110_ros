# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rc110_msgs:msg/Offsets.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Offsets(type):
    """Metaclass of message 'Offsets'."""

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
                'rc110_msgs.msg.Offsets')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__offsets
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__offsets
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__offsets
            cls._TYPE_SUPPORT = module.type_support_msg__msg__offsets
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__offsets

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Offsets(metaclass=Metaclass_Offsets):
    """Message class 'Offsets'."""

    __slots__ = [
        '_gyro',
        '_accel_x',
        '_accel_y',
        '_accel_z',
        '_motor_current',
        '_steering',
    ]

    _fields_and_field_types = {
        'gyro': 'float',
        'accel_x': 'float',
        'accel_y': 'float',
        'accel_z': 'float',
        'motor_current': 'float',
        'steering': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gyro = kwargs.get('gyro', float())
        self.accel_x = kwargs.get('accel_x', float())
        self.accel_y = kwargs.get('accel_y', float())
        self.accel_z = kwargs.get('accel_z', float())
        self.motor_current = kwargs.get('motor_current', float())
        self.steering = kwargs.get('steering', float())

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
        if self.gyro != other.gyro:
            return False
        if self.accel_x != other.accel_x:
            return False
        if self.accel_y != other.accel_y:
            return False
        if self.accel_z != other.accel_z:
            return False
        if self.motor_current != other.motor_current:
            return False
        if self.steering != other.steering:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def gyro(self):
        """Message field 'gyro'."""
        return self._gyro

    @gyro.setter
    def gyro(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gyro' field must be of type 'float'"
        self._gyro = value

    @property
    def accel_x(self):
        """Message field 'accel_x'."""
        return self._accel_x

    @accel_x.setter
    def accel_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accel_x' field must be of type 'float'"
        self._accel_x = value

    @property
    def accel_y(self):
        """Message field 'accel_y'."""
        return self._accel_y

    @accel_y.setter
    def accel_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accel_y' field must be of type 'float'"
        self._accel_y = value

    @property
    def accel_z(self):
        """Message field 'accel_z'."""
        return self._accel_z

    @accel_z.setter
    def accel_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accel_z' field must be of type 'float'"
        self._accel_z = value

    @property
    def motor_current(self):
        """Message field 'motor_current'."""
        return self._motor_current

    @motor_current.setter
    def motor_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor_current' field must be of type 'float'"
        self._motor_current = value

    @property
    def steering(self):
        """Message field 'steering'."""
        return self._steering

    @steering.setter
    def steering(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering' field must be of type 'float'"
        self._steering = value
