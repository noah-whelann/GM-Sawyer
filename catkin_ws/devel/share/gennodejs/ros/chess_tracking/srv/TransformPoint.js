// Auto-generated. Do not edit!

// (in-package chess_tracking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class TransformPointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_point = null;
    }
    else {
      if (initObj.hasOwnProperty('input_point')) {
        this.input_point = initObj.input_point
      }
      else {
        this.input_point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransformPointRequest
    // Serialize message field [input_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.input_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransformPointRequest
    let len;
    let data = new TransformPointRequest(null);
    // Deserialize message field [input_point]
    data.input_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chess_tracking/TransformPointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46e1a079c4b17e399923559c1363c355';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TransformPoint.srv
    geometry_msgs/Point input_point
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransformPointRequest(null);
    if (msg.input_point !== undefined) {
      resolved.input_point = geometry_msgs.msg.Point.Resolve(msg.input_point)
    }
    else {
      resolved.input_point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

class TransformPointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transformed_point = null;
    }
    else {
      if (initObj.hasOwnProperty('transformed_point')) {
        this.transformed_point = initObj.transformed_point
      }
      else {
        this.transformed_point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TransformPointResponse
    // Serialize message field [transformed_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.transformed_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TransformPointResponse
    let len;
    let data = new TransformPointResponse(null);
    // Deserialize message field [transformed_point]
    data.transformed_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chess_tracking/TransformPointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9f5f68d743ae7df9ba120cc24bf01e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point transformed_point
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TransformPointResponse(null);
    if (msg.transformed_point !== undefined) {
      resolved.transformed_point = geometry_msgs.msg.Point.Resolve(msg.transformed_point)
    }
    else {
      resolved.transformed_point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = {
  Request: TransformPointRequest,
  Response: TransformPointResponse,
  md5sum() { return '3ee7093c6df18d4364892f198e4fb79b'; },
  datatype() { return 'chess_tracking/TransformPoint'; }
};
