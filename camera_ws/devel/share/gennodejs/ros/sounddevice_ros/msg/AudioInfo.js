// Auto-generated. Do not edit!

// (in-package sounddevice_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AudioInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_channels = null;
      this.sample_rate = null;
      this.subtype = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_channels')) {
        this.num_channels = initObj.num_channels
      }
      else {
        this.num_channels = 0;
      }
      if (initObj.hasOwnProperty('sample_rate')) {
        this.sample_rate = initObj.sample_rate
      }
      else {
        this.sample_rate = 0;
      }
      if (initObj.hasOwnProperty('subtype')) {
        this.subtype = initObj.subtype
      }
      else {
        this.subtype = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AudioInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_channels]
    bufferOffset = _serializer.int32(obj.num_channels, buffer, bufferOffset);
    // Serialize message field [sample_rate]
    bufferOffset = _serializer.int32(obj.sample_rate, buffer, bufferOffset);
    // Serialize message field [subtype]
    bufferOffset = _serializer.string(obj.subtype, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AudioInfo
    let len;
    let data = new AudioInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_channels]
    data.num_channels = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [sample_rate]
    data.sample_rate = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [subtype]
    data.subtype = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.subtype);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sounddevice_ros/AudioInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd52fdd030548864e37e9bab9114e6549';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 num_channels
    int32 sample_rate
    string subtype
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AudioInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_channels !== undefined) {
      resolved.num_channels = msg.num_channels;
    }
    else {
      resolved.num_channels = 0
    }

    if (msg.sample_rate !== undefined) {
      resolved.sample_rate = msg.sample_rate;
    }
    else {
      resolved.sample_rate = 0
    }

    if (msg.subtype !== undefined) {
      resolved.subtype = msg.subtype;
    }
    else {
      resolved.subtype = ''
    }

    return resolved;
    }
};

module.exports = AudioInfo;
