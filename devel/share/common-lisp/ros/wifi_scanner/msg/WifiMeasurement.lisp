; Auto-generated. Do not edit!


(cl:in-package wifi_scanner-msg)


;//! \htmlinclude WifiMeasurement.msg.html

(cl:defclass <WifiMeasurement> (roslisp-msg-protocol:ros-message)
  ((ssid
    :reader ssid
    :initarg :ssid
    :type cl:string
    :initform "")
   (bssid
    :reader bssid
    :initarg :bssid
    :type cl:string
    :initform "")
   (rssi
    :reader rssi
    :initarg :rssi
    :type cl:float
    :initform 0.0)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass WifiMeasurement (<WifiMeasurement>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiMeasurement>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiMeasurement)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wifi_scanner-msg:<WifiMeasurement> is deprecated: use wifi_scanner-msg:WifiMeasurement instead.")))

(cl:ensure-generic-function 'ssid-val :lambda-list '(m))
(cl:defmethod ssid-val ((m <WifiMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_scanner-msg:ssid-val is deprecated.  Use wifi_scanner-msg:ssid instead.")
  (ssid m))

(cl:ensure-generic-function 'bssid-val :lambda-list '(m))
(cl:defmethod bssid-val ((m <WifiMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_scanner-msg:bssid-val is deprecated.  Use wifi_scanner-msg:bssid instead.")
  (bssid m))

(cl:ensure-generic-function 'rssi-val :lambda-list '(m))
(cl:defmethod rssi-val ((m <WifiMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_scanner-msg:rssi-val is deprecated.  Use wifi_scanner-msg:rssi instead.")
  (rssi m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <WifiMeasurement>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_scanner-msg:stamp-val is deprecated.  Use wifi_scanner-msg:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiMeasurement>) ostream)
  "Serializes a message object of type '<WifiMeasurement>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ssid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ssid))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bssid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bssid))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rssi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiMeasurement>) istream)
  "Deserializes a message object of type '<WifiMeasurement>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ssid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ssid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bssid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bssid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rssi) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiMeasurement>)))
  "Returns string type for a message object of type '<WifiMeasurement>"
  "wifi_scanner/WifiMeasurement")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiMeasurement)))
  "Returns string type for a message object of type 'WifiMeasurement"
  "wifi_scanner/WifiMeasurement")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiMeasurement>)))
  "Returns md5sum for a message object of type '<WifiMeasurement>"
  "50748349ed213168558a6fc822d7f7f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiMeasurement)))
  "Returns md5sum for a message object of type 'WifiMeasurement"
  "50748349ed213168558a6fc822d7f7f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiMeasurement>)))
  "Returns full string definition for message of type '<WifiMeasurement>"
  (cl:format cl:nil "string ssid~%string bssid~%float32 rssi~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiMeasurement)))
  "Returns full string definition for message of type 'WifiMeasurement"
  (cl:format cl:nil "string ssid~%string bssid~%float32 rssi~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiMeasurement>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ssid))
     4 (cl:length (cl:slot-value msg 'bssid))
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiMeasurement>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiMeasurement
    (cl:cons ':ssid (ssid msg))
    (cl:cons ':bssid (bssid msg))
    (cl:cons ':rssi (rssi msg))
    (cl:cons ':stamp (stamp msg))
))
