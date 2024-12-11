// Auto-generated. Do not edit!

// (in-package chess_tracking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class BoardStringRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoardStringRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoardStringRequest
    let len;
    let data = new BoardStringRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chess_tracking/BoardStringRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoardStringRequest(null);
    return resolved;
    }
};

class BoardStringResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoardStringResponse
    // Serialize message field [output]
    bufferOffset = _serializer.string(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoardStringResponse
    let len;
    let data = new BoardStringResponse(null);
    // Deserialize message field [output]
    data.output = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.output);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chess_tracking/BoardStringResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0825d95fdfa2c8f4bbb4e9c74bccd3fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string output
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoardStringResponse(null);
    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: BoardStringRequest,
  Response: BoardStringResponse,
  md5sum() { return '0825d95fdfa2c8f4bbb4e9c74bccd3fd'; },
  datatype() { return 'chess_tracking/BoardString'; }
};
