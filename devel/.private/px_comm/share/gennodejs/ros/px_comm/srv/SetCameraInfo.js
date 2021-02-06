// Auto-generated. Do not edit!

// (in-package px_comm.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CameraInfo = require('../msg/CameraInfo.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetCameraInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camera_info = null;
    }
    else {
      if (initObj.hasOwnProperty('camera_info')) {
        this.camera_info = initObj.camera_info
      }
      else {
        this.camera_info = new CameraInfo();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCameraInfoRequest
    // Serialize message field [camera_info]
    bufferOffset = CameraInfo.serialize(obj.camera_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCameraInfoRequest
    let len;
    let data = new SetCameraInfoRequest(null);
    // Deserialize message field [camera_info]
    data.camera_info = CameraInfo.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CameraInfo.getMessageSize(object.camera_info);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'px_comm/SetCameraInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a9e1a5d43e759a7ba44f5a4ddd63496';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    CameraInfo camera_info
    
    ================================================================================
    MSG: px_comm/CameraInfo
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
    const resolved = new SetCameraInfoRequest(null);
    if (msg.camera_info !== undefined) {
      resolved.camera_info = CameraInfo.Resolve(msg.camera_info)
    }
    else {
      resolved.camera_info = new CameraInfo()
    }

    return resolved;
    }
};

class SetCameraInfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.status_message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('status_message')) {
        this.status_message = initObj.status_message
      }
      else {
        this.status_message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCameraInfoResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCameraInfoResponse
    let len;
    let data = new SetCameraInfoResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.status_message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'px_comm/SetCameraInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ec6f3eff0161f4257b808b12bc830c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string status_message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCameraInfoResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.status_message !== undefined) {
      resolved.status_message = msg.status_message;
    }
    else {
      resolved.status_message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetCameraInfoRequest,
  Response: SetCameraInfoResponse,
  md5sum() { return 'b7b33d05bd0d56b83943d2370771da4c'; },
  datatype() { return 'px_comm/SetCameraInfo'; }
};
