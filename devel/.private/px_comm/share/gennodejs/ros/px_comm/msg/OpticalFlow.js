// Auto-generated. Do not edit!

// (in-package px_comm.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class OpticalFlow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ground_distance = null;
      this.flow_x = null;
      this.flow_y = null;
      this.velocity_x = null;
      this.velocity_y = null;
      this.quality = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ground_distance')) {
        this.ground_distance = initObj.ground_distance
      }
      else {
        this.ground_distance = 0.0;
      }
      if (initObj.hasOwnProperty('flow_x')) {
        this.flow_x = initObj.flow_x
      }
      else {
        this.flow_x = 0;
      }
      if (initObj.hasOwnProperty('flow_y')) {
        this.flow_y = initObj.flow_y
      }
      else {
        this.flow_y = 0;
      }
      if (initObj.hasOwnProperty('velocity_x')) {
        this.velocity_x = initObj.velocity_x
      }
      else {
        this.velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_y')) {
        this.velocity_y = initObj.velocity_y
      }
      else {
        this.velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('quality')) {
        this.quality = initObj.quality
      }
      else {
        this.quality = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OpticalFlow
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ground_distance]
    bufferOffset = _serializer.float32(obj.ground_distance, buffer, bufferOffset);
    // Serialize message field [flow_x]
    bufferOffset = _serializer.int16(obj.flow_x, buffer, bufferOffset);
    // Serialize message field [flow_y]
    bufferOffset = _serializer.int16(obj.flow_y, buffer, bufferOffset);
    // Serialize message field [velocity_x]
    bufferOffset = _serializer.float32(obj.velocity_x, buffer, bufferOffset);
    // Serialize message field [velocity_y]
    bufferOffset = _serializer.float32(obj.velocity_y, buffer, bufferOffset);
    // Serialize message field [quality]
    bufferOffset = _serializer.uint8(obj.quality, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OpticalFlow
    let len;
    let data = new OpticalFlow(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ground_distance]
    data.ground_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [flow_x]
    data.flow_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [flow_y]
    data.flow_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [velocity_x]
    data.velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_y]
    data.velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [quality]
    data.quality = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px_comm/OpticalFlow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6705fe0e94fea14978a508d00cf97427';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32 ground_distance  # distance to ground in meters
    int16   flow_x           # x-component of optical flow in pixels
    int16   flow_y           # y-component of optical flow in pixels
    float32 velocity_x       # x-component of scaled optical flow in m/s
    float32 velocity_y       # y-component of scaled optical flow in m/s
    uint8   quality          # quality of optical flow estimate
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OpticalFlow(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ground_distance !== undefined) {
      resolved.ground_distance = msg.ground_distance;
    }
    else {
      resolved.ground_distance = 0.0
    }

    if (msg.flow_x !== undefined) {
      resolved.flow_x = msg.flow_x;
    }
    else {
      resolved.flow_x = 0
    }

    if (msg.flow_y !== undefined) {
      resolved.flow_y = msg.flow_y;
    }
    else {
      resolved.flow_y = 0
    }

    if (msg.velocity_x !== undefined) {
      resolved.velocity_x = msg.velocity_x;
    }
    else {
      resolved.velocity_x = 0.0
    }

    if (msg.velocity_y !== undefined) {
      resolved.velocity_y = msg.velocity_y;
    }
    else {
      resolved.velocity_y = 0.0
    }

    if (msg.quality !== undefined) {
      resolved.quality = msg.quality;
    }
    else {
      resolved.quality = 0
    }

    return resolved;
    }
};

module.exports = OpticalFlow;
