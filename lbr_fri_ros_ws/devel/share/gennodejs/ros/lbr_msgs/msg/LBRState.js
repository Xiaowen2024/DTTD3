// Auto-generated. Do not edit!

// (in-package lbr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LBRState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.sample_time = null;
      this.position = null;
      this.torque = null;
      this.external_torque = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = new std_msgs.msg.Time();
      }
      if (initObj.hasOwnProperty('sample_time')) {
        this.sample_time = initObj.sample_time
      }
      else {
        this.sample_time = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = [];
      }
      if (initObj.hasOwnProperty('external_torque')) {
        this.external_torque = initObj.external_torque
      }
      else {
        this.external_torque = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LBRState
    // Serialize message field [stamp]
    bufferOffset = std_msgs.msg.Time.serialize(obj.stamp, buffer, bufferOffset);
    // Serialize message field [sample_time]
    bufferOffset = _serializer.float64(obj.sample_time, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float64(obj.position, buffer, bufferOffset, null);
    // Serialize message field [torque]
    bufferOffset = _arraySerializer.float64(obj.torque, buffer, bufferOffset, null);
    // Serialize message field [external_torque]
    bufferOffset = _arraySerializer.float64(obj.external_torque, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LBRState
    let len;
    let data = new LBRState(null);
    // Deserialize message field [stamp]
    data.stamp = std_msgs.msg.Time.deserialize(buffer, bufferOffset);
    // Deserialize message field [sample_time]
    data.sample_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [torque]
    data.torque = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [external_torque]
    data.external_torque = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.position.length;
    length += 8 * object.torque.length;
    length += 8 * object.external_torque.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lbr_msgs/LBRState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '71e053993298b574a690156a73db8a42';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Time stamp
    float64 sample_time
    float64[] position
    float64[] torque
    float64[] external_torque
    
    ================================================================================
    MSG: std_msgs/Time
    time data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LBRState(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = std_msgs.msg.Time.Resolve(msg.stamp)
    }
    else {
      resolved.stamp = new std_msgs.msg.Time()
    }

    if (msg.sample_time !== undefined) {
      resolved.sample_time = msg.sample_time;
    }
    else {
      resolved.sample_time = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = []
    }

    if (msg.external_torque !== undefined) {
      resolved.external_torque = msg.external_torque;
    }
    else {
      resolved.external_torque = []
    }

    return resolved;
    }
};

module.exports = LBRState;
