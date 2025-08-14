# generated from rosidl_generator_py/resource/_idl.py.em
# with input from torsobot_interfaces:srv/RequestControlParams.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RequestControlParams_Request(type):
    """Metaclass of message 'RequestControlParams_Request'."""

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
                'torsobot_interfaces.srv.RequestControlParams_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__request_control_params__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__request_control_params__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__request_control_params__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__request_control_params__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__request_control_params__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RequestControlParams_Request(metaclass=Metaclass_RequestControlParams_Request):
    """Message class 'RequestControlParams_Request'."""

    __slots__ = [
        '_check_fields',
    ]

    _fields_and_field_types = {
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_RequestControlParams_Response(type):
    """Metaclass of message 'RequestControlParams_Response'."""

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
                'torsobot_interfaces.srv.RequestControlParams_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__request_control_params__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__request_control_params__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__request_control_params__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__request_control_params__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__request_control_params__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RequestControlParams_Response(metaclass=Metaclass_RequestControlParams_Response):
    """Message class 'RequestControlParams_Response'."""

    __slots__ = [
        '_desired_torso_pitch',
        '_mot_max_torque',
        '_kp',
        '_ki',
        '_kd',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'desired_torso_pitch': 'double',
        'mot_max_torque': 'double',
        'kp': 'double',
        'ki': 'double',
        'kd': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
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
        self.desired_torso_pitch = kwargs.get('desired_torso_pitch', float())
        self.mot_max_torque = kwargs.get('mot_max_torque', float())
        self.kp = kwargs.get('kp', float())
        self.ki = kwargs.get('ki', float())
        self.kd = kwargs.get('kd', float())

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
        if self.desired_torso_pitch != other.desired_torso_pitch:
            return False
        if self.mot_max_torque != other.mot_max_torque:
            return False
        if self.kp != other.kp:
            return False
        if self.ki != other.ki:
            return False
        if self.kd != other.kd:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def desired_torso_pitch(self):
        """Message field 'desired_torso_pitch'."""
        return self._desired_torso_pitch

    @desired_torso_pitch.setter
    def desired_torso_pitch(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'desired_torso_pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'desired_torso_pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._desired_torso_pitch = value

    @builtins.property
    def mot_max_torque(self):
        """Message field 'mot_max_torque'."""
        return self._mot_max_torque

    @mot_max_torque.setter
    def mot_max_torque(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'mot_max_torque' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'mot_max_torque' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._mot_max_torque = value

    @builtins.property
    def kp(self):
        """Message field 'kp'."""
        return self._kp

    @kp.setter
    def kp(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'kp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'kp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._kp = value

    @builtins.property
    def ki(self):
        """Message field 'ki'."""
        return self._ki

    @ki.setter
    def ki(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ki' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ki' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ki = value

    @builtins.property
    def kd(self):
        """Message field 'kd'."""
        return self._kd

    @kd.setter
    def kd(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'kd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'kd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._kd = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_RequestControlParams_Event(type):
    """Metaclass of message 'RequestControlParams_Event'."""

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
                'torsobot_interfaces.srv.RequestControlParams_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__request_control_params__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__request_control_params__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__request_control_params__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__request_control_params__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__request_control_params__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RequestControlParams_Event(metaclass=Metaclass_RequestControlParams_Event):
    """Message class 'RequestControlParams_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<torsobot_interfaces/RequestControlParams_Request, 1>',
        'response': 'sequence<torsobot_interfaces/RequestControlParams_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['torsobot_interfaces', 'srv'], 'RequestControlParams_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['torsobot_interfaces', 'srv'], 'RequestControlParams_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from torsobot_interfaces.srv import RequestControlParams_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, RequestControlParams_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'RequestControlParams_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from torsobot_interfaces.srv import RequestControlParams_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, RequestControlParams_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'RequestControlParams_Response'"
        self._response = value


class Metaclass_RequestControlParams(type):
    """Metaclass of service 'RequestControlParams'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('torsobot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'torsobot_interfaces.srv.RequestControlParams')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__request_control_params

            from torsobot_interfaces.srv import _request_control_params
            if _request_control_params.Metaclass_RequestControlParams_Request._TYPE_SUPPORT is None:
                _request_control_params.Metaclass_RequestControlParams_Request.__import_type_support__()
            if _request_control_params.Metaclass_RequestControlParams_Response._TYPE_SUPPORT is None:
                _request_control_params.Metaclass_RequestControlParams_Response.__import_type_support__()
            if _request_control_params.Metaclass_RequestControlParams_Event._TYPE_SUPPORT is None:
                _request_control_params.Metaclass_RequestControlParams_Event.__import_type_support__()


class RequestControlParams(metaclass=Metaclass_RequestControlParams):
    from torsobot_interfaces.srv._request_control_params import RequestControlParams_Request as Request
    from torsobot_interfaces.srv._request_control_params import RequestControlParams_Response as Response
    from torsobot_interfaces.srv._request_control_params import RequestControlParams_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
