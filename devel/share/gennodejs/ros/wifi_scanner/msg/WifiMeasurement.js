// Auto-generated. Do not edit!

// (in-package wifi_scanner.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class WifiMeasurement {
  constructor() {
    this.ssid = '';
    this.bssid = '';
    this.rssi = 0.0;
    this.stamp = {secs: 0, nsecs: 0};
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type WifiMeasurement
    // Serialize message field [ssid]
    bufferInfo = _serializer.string(obj.ssid, bufferInfo);
    // Serialize message field [bssid]
    bufferInfo = _serializer.string(obj.bssid, bufferInfo);
    // Serialize message field [rssi]
    bufferInfo = _serializer.float32(obj.rssi, bufferInfo);
    // Serialize message field [stamp]
    bufferInfo = _serializer.time(obj.stamp, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type WifiMeasurement
    let tmp;
    let len;
    let data = new WifiMeasurement();
    // Deserialize message field [ssid]
    tmp = _deserializer.string(buffer);
    data.ssid = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [bssid]
    tmp = _deserializer.string(buffer);
    data.bssid = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [rssi]
    tmp = _deserializer.float32(buffer);
    data.rssi = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [stamp]
    tmp = _deserializer.time(buffer);
    data.stamp = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'wifi_scanner/WifiMeasurement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50748349ed213168558a6fc822d7f7f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string ssid
    string bssid
    float32 rssi
    time stamp
    
    `;
  }

};

module.exports = WifiMeasurement;
