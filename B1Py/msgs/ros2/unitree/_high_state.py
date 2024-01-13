# generated from rosidl_generator_py/resource/_idl.py.em
# with input from unitree_msgs:msg/HighState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'head'
# Member 'sn'
# Member 'version'
# Member 'foot_force'
# Member 'foot_force_est'
# Member 'position'
# Member 'velocity'
# Member 'range_obstacle'
# Member 'wireless_remote'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HighState(type):
    """Metaclass of message 'HighState'."""

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
            module = import_type_support('unitree_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'unitree_msgs.msg.HighState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__high_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__high_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__high_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__high_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__high_state

            from B1Py.msgs.ros2.unitree import BmsState
            if BmsState.__class__._TYPE_SUPPORT is None:
                BmsState.__class__.__import_type_support__()

            from B1Py.msgs.ros2.unitree import Cartesian
            if Cartesian.__class__._TYPE_SUPPORT is None:
                Cartesian.__class__.__import_type_support__()

            from B1Py.msgs.ros2.unitree import IMU
            if IMU.__class__._TYPE_SUPPORT is None:
                IMU.__class__.__import_type_support__()

            from B1Py.msgs.ros2.unitree import MotorState
            if MotorState.__class__._TYPE_SUPPORT is None:
                MotorState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HighState(metaclass=Metaclass_HighState):
    """Message class 'HighState'."""

    __slots__ = [
        '_head',
        '_level_flag',
        '_frame_reserve',
        '_sn',
        '_version',
        '_band_width',
        '_imu',
        '_motor_state',
        '_bms',
        '_foot_force',
        '_foot_force_est',
        '_mode',
        '_progress',
        '_gait_type',
        '_foot_raise_height',
        '_position',
        '_body_height',
        '_velocity',
        '_yaw_speed',
        '_range_obstacle',
        '_foot_position2body',
        '_foot_speed2body',
        '_wireless_remote',
        '_reserve',
        '_crc',
    ]

    _fields_and_field_types = {
        'head': 'uint8[2]',
        'level_flag': 'uint8',
        'frame_reserve': 'uint8',
        'sn': 'uint32[2]',
        'version': 'uint32[2]',
        'band_width': 'uint16',
        'imu': 'unitree_msgs/IMU',
        'motor_state': 'unitree_msgs/MotorState[20]',
        'bms': 'unitree_msgs/BmsState',
        'foot_force': 'int16[4]',
        'foot_force_est': 'int16[4]',
        'mode': 'uint8',
        'progress': 'float',
        'gait_type': 'uint8',
        'foot_raise_height': 'float',
        'position': 'float[3]',
        'body_height': 'float',
        'velocity': 'float[3]',
        'yaw_speed': 'float',
        'range_obstacle': 'float[4]',
        'foot_position2body': 'unitree_msgs/Cartesian[4]',
        'foot_speed2body': 'unitree_msgs/Cartesian[4]',
        'wireless_remote': 'uint8[40]',
        'reserve': 'uint32',
        'crc': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 2),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint32'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_msgs', 'msg'], 'IMU'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['unitree_msgs', 'msg'], 'MotorState'), 20),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['unitree_msgs', 'msg'], 'BmsState'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['unitree_msgs', 'msg'], 'Cartesian'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['unitree_msgs', 'msg'], 'Cartesian'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 40),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'head' not in kwargs:
            self.head = numpy.zeros(2, dtype=numpy.uint8)
        else:
            self.head = numpy.array(kwargs.get('head'), dtype=numpy.uint8)
            assert self.head.shape == (2, )
        self.level_flag = kwargs.get('level_flag', int())
        self.frame_reserve = kwargs.get('frame_reserve', int())
        if 'sn' not in kwargs:
            self.sn = numpy.zeros(2, dtype=numpy.uint32)
        else:
            self.sn = numpy.array(kwargs.get('sn'), dtype=numpy.uint32)
            assert self.sn.shape == (2, )
        if 'version' not in kwargs:
            self.version = numpy.zeros(2, dtype=numpy.uint32)
        else:
            self.version = numpy.array(kwargs.get('version'), dtype=numpy.uint32)
            assert self.version.shape == (2, )
        self.band_width = kwargs.get('band_width', int())
        from B1Py.msgs.ros2.unitree import IMU
        self.imu = kwargs.get('imu', IMU())
        from B1Py.msgs.ros2.unitree import MotorState
        self.motor_state = kwargs.get(
            'motor_state',
            [MotorState() for x in range(20)]
        )
        from B1Py.msgs.ros2.unitree import BmsState
        self.bms = kwargs.get('bms', BmsState())
        if 'foot_force' not in kwargs:
            self.foot_force = numpy.zeros(4, dtype=numpy.int16)
        else:
            self.foot_force = numpy.array(kwargs.get('foot_force'), dtype=numpy.int16)
            assert self.foot_force.shape == (4, )
        if 'foot_force_est' not in kwargs:
            self.foot_force_est = numpy.zeros(4, dtype=numpy.int16)
        else:
            self.foot_force_est = numpy.array(kwargs.get('foot_force_est'), dtype=numpy.int16)
            assert self.foot_force_est.shape == (4, )
        self.mode = kwargs.get('mode', int())
        self.progress = kwargs.get('progress', float())
        self.gait_type = kwargs.get('gait_type', int())
        self.foot_raise_height = kwargs.get('foot_raise_height', float())
        if 'position' not in kwargs:
            self.position = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.position = numpy.array(kwargs.get('position'), dtype=numpy.float32)
            assert self.position.shape == (3, )
        self.body_height = kwargs.get('body_height', float())
        if 'velocity' not in kwargs:
            self.velocity = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.velocity = numpy.array(kwargs.get('velocity'), dtype=numpy.float32)
            assert self.velocity.shape == (3, )
        self.yaw_speed = kwargs.get('yaw_speed', float())
        if 'range_obstacle' not in kwargs:
            self.range_obstacle = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.range_obstacle = numpy.array(kwargs.get('range_obstacle'), dtype=numpy.float32)
            assert self.range_obstacle.shape == (4, )
        from B1Py.msgs.ros2.unitree import Cartesian
        self.foot_position2body = kwargs.get(
            'foot_position2body',
            [Cartesian() for x in range(4)]
        )
        from B1Py.msgs.ros2.unitree import Cartesian
        self.foot_speed2body = kwargs.get(
            'foot_speed2body',
            [Cartesian() for x in range(4)]
        )
        if 'wireless_remote' not in kwargs:
            self.wireless_remote = numpy.zeros(40, dtype=numpy.uint8)
        else:
            self.wireless_remote = numpy.array(kwargs.get('wireless_remote'), dtype=numpy.uint8)
            assert self.wireless_remote.shape == (40, )
        self.reserve = kwargs.get('reserve', int())
        self.crc = kwargs.get('crc', int())

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
        if all(self.head != other.head):
            return False
        if self.level_flag != other.level_flag:
            return False
        if self.frame_reserve != other.frame_reserve:
            return False
        if all(self.sn != other.sn):
            return False
        if all(self.version != other.version):
            return False
        if self.band_width != other.band_width:
            return False
        if self.imu != other.imu:
            return False
        if self.motor_state != other.motor_state:
            return False
        if self.bms != other.bms:
            return False
        if all(self.foot_force != other.foot_force):
            return False
        if all(self.foot_force_est != other.foot_force_est):
            return False
        if self.mode != other.mode:
            return False
        if self.progress != other.progress:
            return False
        if self.gait_type != other.gait_type:
            return False
        if self.foot_raise_height != other.foot_raise_height:
            return False
        if all(self.position != other.position):
            return False
        if self.body_height != other.body_height:
            return False
        if all(self.velocity != other.velocity):
            return False
        if self.yaw_speed != other.yaw_speed:
            return False
        if all(self.range_obstacle != other.range_obstacle):
            return False
        if self.foot_position2body != other.foot_position2body:
            return False
        if self.foot_speed2body != other.foot_speed2body:
            return False
        if all(self.wireless_remote != other.wireless_remote):
            return False
        if self.reserve != other.reserve:
            return False
        if self.crc != other.crc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def head(self):
        """Message field 'head'."""
        return self._head

    @head.setter
    def head(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'head' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 2, \
                "The 'head' numpy.ndarray() must have a size of 2"
            self._head = value
            return
        if __debug__:
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'head' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._head = numpy.array(value, dtype=numpy.uint8)

    @property
    def level_flag(self):
        """Message field 'level_flag'."""
        return self._level_flag

    @level_flag.setter
    def level_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'level_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'level_flag' field must be an unsigned integer in [0, 255]"
        self._level_flag = value

    @property
    def frame_reserve(self):
        """Message field 'frame_reserve'."""
        return self._frame_reserve

    @frame_reserve.setter
    def frame_reserve(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame_reserve' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'frame_reserve' field must be an unsigned integer in [0, 255]"
        self._frame_reserve = value

    @property
    def sn(self):
        """Message field 'sn'."""
        return self._sn

    @sn.setter
    def sn(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'sn' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 2, \
                "The 'sn' numpy.ndarray() must have a size of 2"
            self._sn = value
            return
        if __debug__:
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'sn' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._sn = numpy.array(value, dtype=numpy.uint32)

    @property
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint32, \
                "The 'version' numpy.ndarray() must have the dtype of 'numpy.uint32'"
            assert value.size == 2, \
                "The 'version' numpy.ndarray() must have a size of 2"
            self._version = value
            return
        if __debug__:
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'version' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._version = numpy.array(value, dtype=numpy.uint32)

    @property
    def band_width(self):
        """Message field 'band_width'."""
        return self._band_width

    @band_width.setter
    def band_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'band_width' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'band_width' field must be an unsigned integer in [0, 65535]"
        self._band_width = value

    @property
    def imu(self):
        """Message field 'imu'."""
        return self._imu

    @imu.setter
    def imu(self, value):
        if __debug__:
            from B1Py.msgs.ros2.unitree import IMU
            assert \
                isinstance(value, IMU), \
                "The 'imu' field must be a sub message of type 'IMU'"
        self._imu = value

    @property
    def motor_state(self):
        """Message field 'motor_state'."""
        return self._motor_state

    @motor_state.setter
    def motor_state(self, value):
        if __debug__:
            from B1Py.msgs.ros2.unitree import MotorState
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
                 len(value) == 20 and
                 all(isinstance(v, MotorState) for v in value) and
                 True), \
                "The 'motor_state' field must be a set or sequence with length 20 and each value of type 'MotorState'"
        self._motor_state = value

    @property
    def bms(self):
        """Message field 'bms'."""
        return self._bms

    @bms.setter
    def bms(self, value):
        if __debug__:
            from B1Py.msgs.ros2.unitree import BmsState
            assert \
                isinstance(value, BmsState), \
                "The 'bms' field must be a sub message of type 'BmsState'"
        self._bms = value

    @property
    def foot_force(self):
        """Message field 'foot_force'."""
        return self._foot_force

    @foot_force.setter
    def foot_force(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'foot_force' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 4, \
                "The 'foot_force' numpy.ndarray() must have a size of 4"
            self._foot_force = value
            return
        if __debug__:
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
                 len(value) == 4 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'foot_force' field must be a set or sequence with length 4 and each value of type 'int' and each integer in [-32768, 32767]"
        self._foot_force = numpy.array(value, dtype=numpy.int16)

    @property
    def foot_force_est(self):
        """Message field 'foot_force_est'."""
        return self._foot_force_est

    @foot_force_est.setter
    def foot_force_est(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'foot_force_est' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 4, \
                "The 'foot_force_est' numpy.ndarray() must have a size of 4"
            self._foot_force_est = value
            return
        if __debug__:
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
                 len(value) == 4 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'foot_force_est' field must be a set or sequence with length 4 and each value of type 'int' and each integer in [-32768, 32767]"
        self._foot_force_est = numpy.array(value, dtype=numpy.int16)

    @property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @property
    def progress(self):
        """Message field 'progress'."""
        return self._progress

    @progress.setter
    def progress(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'progress' field must be of type 'float'"
        self._progress = value

    @property
    def gait_type(self):
        """Message field 'gait_type'."""
        return self._gait_type

    @gait_type.setter
    def gait_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gait_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gait_type' field must be an unsigned integer in [0, 255]"
        self._gait_type = value

    @property
    def foot_raise_height(self):
        """Message field 'foot_raise_height'."""
        return self._foot_raise_height

    @foot_raise_height.setter
    def foot_raise_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'foot_raise_height' field must be of type 'float'"
        self._foot_raise_height = value

    @property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'position' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'position' numpy.ndarray() must have a size of 3"
            self._position = value
            return
        if __debug__:
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'position' field must be a set or sequence with length 3 and each value of type 'float'"
        self._position = numpy.array(value, dtype=numpy.float32)

    @property
    def body_height(self):
        """Message field 'body_height'."""
        return self._body_height

    @body_height.setter
    def body_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'body_height' field must be of type 'float'"
        self._body_height = value

    @property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'velocity' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'velocity' numpy.ndarray() must have a size of 3"
            self._velocity = value
            return
        if __debug__:
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'velocity' field must be a set or sequence with length 3 and each value of type 'float'"
        self._velocity = numpy.array(value, dtype=numpy.float32)

    @property
    def yaw_speed(self):
        """Message field 'yaw_speed'."""
        return self._yaw_speed

    @yaw_speed.setter
    def yaw_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_speed' field must be of type 'float'"
        self._yaw_speed = value

    @property
    def range_obstacle(self):
        """Message field 'range_obstacle'."""
        return self._range_obstacle

    @range_obstacle.setter
    def range_obstacle(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'range_obstacle' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'range_obstacle' numpy.ndarray() must have a size of 4"
            self._range_obstacle = value
            return
        if __debug__:
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
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'range_obstacle' field must be a set or sequence with length 4 and each value of type 'float'"
        self._range_obstacle = numpy.array(value, dtype=numpy.float32)

    @property
    def foot_position2body(self):
        """Message field 'foot_position2body'."""
        return self._foot_position2body

    @foot_position2body.setter
    def foot_position2body(self, value):
        if __debug__:
            from B1Py.msgs.ros2.unitree import Cartesian
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
                 len(value) == 4 and
                 all(isinstance(v, Cartesian) for v in value) and
                 True), \
                "The 'foot_position2body' field must be a set or sequence with length 4 and each value of type 'Cartesian'"
        self._foot_position2body = value

    @property
    def foot_speed2body(self):
        """Message field 'foot_speed2body'."""
        return self._foot_speed2body

    @foot_speed2body.setter
    def foot_speed2body(self, value):
        if __debug__:
            from B1Py.msgs.ros2.unitree import Cartesian
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
                 len(value) == 4 and
                 all(isinstance(v, Cartesian) for v in value) and
                 True), \
                "The 'foot_speed2body' field must be a set or sequence with length 4 and each value of type 'Cartesian'"
        self._foot_speed2body = value

    @property
    def wireless_remote(self):
        """Message field 'wireless_remote'."""
        return self._wireless_remote

    @wireless_remote.setter
    def wireless_remote(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'wireless_remote' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 40, \
                "The 'wireless_remote' numpy.ndarray() must have a size of 40"
            self._wireless_remote = value
            return
        if __debug__:
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
                 len(value) == 40 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'wireless_remote' field must be a set or sequence with length 40 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._wireless_remote = numpy.array(value, dtype=numpy.uint8)

    @property
    def reserve(self):
        """Message field 'reserve'."""
        return self._reserve

    @reserve.setter
    def reserve(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reserve' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'reserve' field must be an unsigned integer in [0, 4294967295]"
        self._reserve = value

    @property
    def crc(self):
        """Message field 'crc'."""
        return self._crc

    @crc.setter
    def crc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'crc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'crc' field must be an unsigned integer in [0, 4294967295]"
        self._crc = value
