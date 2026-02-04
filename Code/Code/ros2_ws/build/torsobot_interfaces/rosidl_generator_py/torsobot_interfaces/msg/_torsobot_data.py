# generated from rosidl_generator_py/resource/_idl.py.em
# with input from torsobot_interfaces:msg/TorsobotData.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TorsobotData(type):
    """Metaclass of message 'TorsobotData'."""

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
            module = import_type_support('torsobot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'torsobot_interfaces.msg.TorsobotData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__torsobot_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__torsobot_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__torsobot_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__torsobot_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__torsobot_data

            from torsobot_interfaces.msg import TorsobotState
            if TorsobotState.__class__._TYPE_SUPPORT is None:
                TorsobotState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TorsobotData(metaclass=Metaclass_TorsobotData):
    """Message class 'TorsobotData'."""

    __slots__ = [
        '_torsobot_state',
        '_wheel_torque',
        '_wheel_cmd_torque',
        '_mot_drv_mode',
        '_torso_pitch_init',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'torsobot_state': 'torsobot_interfaces/TorsobotState',
        'wheel_torque': 'double',
        'wheel_cmd_torque': 'double',
        'mot_drv_mode': 'int8',
        'torso_pitch_init': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['torsobot_interfaces', 'msg'], 'TorsobotState'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from torsobot_interfaces.msg import TorsobotState
        self.torsobot_state = kwargs.get('torsobot_state', TorsobotState())
        self.wheel_torque = kwargs.get('wheel_torque', float())
        self.wheel_cmd_torque = kwargs.get('wheel_cmd_torque', float())
        self.mot_drv_mode = kwargs.get('mot_drv_mode', int())
        self.torso_pitch_init = kwargs.get('torso_pitch_init', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.torsobot_state != other.torsobot_state:
            return False
        if self.wheel_torque != other.wheel_torque:
            return False
        if self.wheel_cmd_torque != other.wheel_cmd_torque:
            return False
        if self.mot_drv_mode != other.mot_drv_mode:
            return False
        if self.torso_pitch_init != other.torso_pitch_init:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def torsobot_state(self):
        """Message field 'torsobot_state'."""
        return self._torsobot_state

    @torsobot_state.setter
    def torsobot_state(self, value):
        if self._check_fields:
            from torsobot_interfaces.msg import TorsobotState
            assert \
                isinstance(value, TorsobotState), \
                "The 'torsobot_state' field must be a sub message of type 'TorsobotState'"
        self._torsobot_state = value

    @builtins.property
    def wheel_torque(self):
        """Message field 'wheel_torque'."""
        return self._wheel_torque

    @wheel_torque.setter
    def wheel_torque(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'wheel_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'wheel_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._wheel_torque = value

    @builtins.property
    def wheel_cmd_torque(self):
        """Message field 'wheel_cmd_torque'."""
        return self._wheel_cmd_torque

    @wheel_cmd_torque.setter
    def wheel_cmd_torque(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'wheel_cmd_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'wheel_cmd_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._wheel_cmd_torque = value

    @builtins.property
    def mot_drv_mode(self):
        """Message field 'mot_drv_mode'."""
        return self._mot_drv_mode

    @mot_drv_mode.setter
    def mot_drv_mode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'mot_drv_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mot_drv_mode' field must be an integer in [-128, 127]"
        self._mot_drv_mode = value

    @builtins.property
    def torso_pitch_init(self):
        """Message field 'torso_pitch_init'."""
        return self._torso_pitch_init

    @torso_pitch_init.setter
    def torso_pitch_init(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'torso_pitch_init' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'torso_pitch_init' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._torso_pitch_init = value
