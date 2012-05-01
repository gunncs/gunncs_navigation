"""autogenerated by genmsg_py from Line.msg. Do not edit."""
import roslib.message
import struct


class Line(roslib.message.Message):
  _md5sum = "8deef184fcc8c22d82c1ccdbad4e8edf"
  _type = "gunncs_navigation_msgs/Line"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
float32 slope
float32 y_intercept

"""
  __slots__ = ['slope','y_intercept']
  _slot_types = ['float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       slope,y_intercept
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Line, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.slope is None:
        self.slope = 0.
      if self.y_intercept is None:
        self.y_intercept = 0.
    else:
      self.slope = 0.
      self.y_intercept = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2f.pack(_x.slope, _x.y_intercept))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.slope, _x.y_intercept,) = _struct_2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2f.pack(_x.slope, _x.y_intercept))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.slope, _x.y_intercept,) = _struct_2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2f = struct.Struct("<2f")
