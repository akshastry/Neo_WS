// Auto-generated. Do not edit!

// (in-package px_comm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CameraInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.camera_model = null;
      this.camera_name = null;
      this.camera_type = null;
      this.image_width = null;
      this.image_height = null;
      this.D = null;
      this.P = null;
      this.M = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('camera_model')) {
        this.camera_model = initObj.camera_model
      }
      else {
        this.camera_model = '';
      }
      if (initObj.hasOwnProperty('camera_name')) {
        this.camera_name = initObj.camera_name
      }
      else {
        this.camera_name = '';
      }
      if (initObj.hasOwnProperty('camera_type')) {
        this.camera_type = initObj.camera_type
      }
      else {
        this.camera_type = '';
      }
      if (initObj.hasOwnProperty('image_width')) {
        this.image_width = initObj.image_width
      }
      else {
        this.image_width = 0;
      }
      if (initObj.hasOwnProperty('image_height')) {
        this.image_height = initObj.image_height
      }
      else {
        this.image_height = 0;
      }
      if (initObj.hasOwnProperty('D')) {
        this.D = initObj.D
      }
      else {
        this.D = [];
      }
      if (initObj.hasOwnProperty('P')) {
        this.P = initObj.P
      }
      else {
        this.P = [];
      }
      if (initObj.hasOwnProperty('M')) {
        this.M = initObj.M
      }
      else {
        this.M = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CameraInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [camera_model]
    bufferOffset = _serializer.string(obj.camera_model, buffer, bufferOffset);
    // Serialize message field [camera_name]
    bufferOffset = _serializer.string(obj.camera_name, buffer, bufferOffset);
    // Serialize message field [camera_type]
    bufferOffset = _serializer.string(obj.camera_type, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.uint32(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.uint32(obj.image_height, buffer, bufferOffset);
    // Serialize message field [D]
    bufferOffset = _arraySerializer.float64(obj.D, buffer, bufferOffset, null);
    // Serialize message field [P]
    bufferOffset = _arraySerializer.float64(obj.P, buffer, bufferOffset, null);
    // Serialize message field [M]
    bufferOffset = _arraySerializer.float64(obj.M, buffer, bufferOffset, null);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CameraInfo
    let len;
    let data = new CameraInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera_model]
    data.camera_model = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [camera_name]
    data.camera_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [camera_type]
    data.camera_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [D]
    data.D = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [P]
    data.P = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [M]
    data.M = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.camera_model.length;
    length += object.camera_name.length;
    length += object.camera_type.length;
    length += 8 * object.D.length;
    length += 8 * object.P.length;
    length += 8 * object.M.length;
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px_comm/CameraInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '014513fdee9cefabe3cec97bca5e5c57';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #######################################################################
    #                     Image acquisition info                          #
    #######################################################################
    
    # Time of image acquisition, camera coordinate frame ID
    Header header    # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into the plane of the image
    
    
    #######################################################################
    #                      Calibration Parameters                         #
    #######################################################################
    # These are fixed during camera calibration. Their values will be the #
    # same in all messages until the camera is recalibrated. Note that    #
    # self-calibrating systems may "recalibrate" frequently.              #
    #######################################################################
    
    # The camera model used.
    string camera_model
    
    # The name of the camera.
    string camera_name
    
    # The type of the camera.
    string camera_type
    
    # The image dimensions with which the camera was calibrated. Normally
    # this will be the full camera resolution in pixels.
    uint32 image_width
    uint32 image_height
    
    # The distortion parameters, size depending on the distortion model.
    float64[] D
    
    # The projection parameters, size depending on the projection model.
    float64[] P
    
    # Other parameters which are not defined by either the distortion or
    # projection model.
    float64[] M
    
    # Pose of camera with respect to a specific reference frame.
    geometry_msgs/Pose pose
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CameraInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.camera_model !== undefined) {
      resolved.camera_model = msg.camera_model;
    }
    else {
      resolved.camera_model = ''
    }

    if (msg.camera_name !== undefined) {
      resolved.camera_name = msg.camera_name;
    }
    else {
      resolved.camera_name = ''
    }

    if (msg.camera_type !== undefined) {
      resolved.camera_type = msg.camera_type;
    }
    else {
      resolved.camera_type = ''
    }

    if (msg.image_width !== undefined) {
      resolved.image_width = msg.image_width;
    }
    else {
      resolved.image_width = 0
    }

    if (msg.image_height !== undefined) {
      resolved.image_height = msg.image_height;
    }
    else {
      resolved.image_height = 0
    }

    if (msg.D !== undefined) {
      resolved.D = msg.D;
    }
    else {
      resolved.D = []
    }

    if (msg.P !== undefined) {
      resolved.P = msg.P;
    }
    else {
      resolved.P = []
    }

    if (msg.M !== undefined) {
      resolved.M = msg.M;
    }
    else {
      resolved.M = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = CameraInfo;
