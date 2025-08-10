// Auto-generated. Do not edit!

// (in-package mission_node.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Bounding_box {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.detected_class = null;
      this.cx = null;
      this.cy = null;
    }
    else {
      if (initObj.hasOwnProperty('detected_class')) {
        this.detected_class = initObj.detected_class
      }
      else {
        this.detected_class = '';
      }
      if (initObj.hasOwnProperty('cx')) {
        this.cx = initObj.cx
      }
      else {
        this.cx = 0;
      }
      if (initObj.hasOwnProperty('cy')) {
        this.cy = initObj.cy
      }
      else {
        this.cy = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Bounding_box
    // Serialize message field [detected_class]
    bufferOffset = _serializer.string(obj.detected_class, buffer, bufferOffset);
    // Serialize message field [cx]
    bufferOffset = _serializer.int16(obj.cx, buffer, bufferOffset);
    // Serialize message field [cy]
    bufferOffset = _serializer.int16(obj.cy, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Bounding_box
    let len;
    let data = new Bounding_box(null);
    // Deserialize message field [detected_class]
    data.detected_class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cx]
    data.cx = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [cy]
    data.cy = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.detected_class);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mission_node/Bounding_box';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '899615dfe4b53d2fa5fba3dfbb98944a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string detected_class
    int16 cx
    int16 cy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Bounding_box(null);
    if (msg.detected_class !== undefined) {
      resolved.detected_class = msg.detected_class;
    }
    else {
      resolved.detected_class = ''
    }

    if (msg.cx !== undefined) {
      resolved.cx = msg.cx;
    }
    else {
      resolved.cx = 0
    }

    if (msg.cy !== undefined) {
      resolved.cy = msg.cy;
    }
    else {
      resolved.cy = 0
    }

    return resolved;
    }
};

module.exports = Bounding_box;
