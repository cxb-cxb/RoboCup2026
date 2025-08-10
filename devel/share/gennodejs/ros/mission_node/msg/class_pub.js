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

class class_pub {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.classify_class = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('classify_class')) {
        this.classify_class = initObj.classify_class
      }
      else {
        this.classify_class = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type class_pub
    // Serialize message field [classify_class]
    bufferOffset = _serializer.string(obj.classify_class, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.int16(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type class_pub
    let len;
    let data = new class_pub(null);
    // Deserialize message field [classify_class]
    data.classify_class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.classify_class);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mission_node/class_pub';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '236eed931e14e0bcfce3c6d0f10d08fb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string classify_class
    int16 confidence
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new class_pub(null);
    if (msg.classify_class !== undefined) {
      resolved.classify_class = msg.classify_class;
    }
    else {
      resolved.classify_class = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0
    }

    return resolved;
    }
};

module.exports = class_pub;
