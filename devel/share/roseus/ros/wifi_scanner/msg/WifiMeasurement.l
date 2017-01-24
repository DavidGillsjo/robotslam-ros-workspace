;; Auto-generated. Do not edit!


(when (boundp 'wifi_scanner::WifiMeasurement)
  (if (not (find-package "WIFI_SCANNER"))
    (make-package "WIFI_SCANNER"))
  (shadow 'WifiMeasurement (find-package "WIFI_SCANNER")))
(unless (find-package "WIFI_SCANNER::WIFIMEASUREMENT")
  (make-package "WIFI_SCANNER::WIFIMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude WifiMeasurement.msg.html


(defclass wifi_scanner::WifiMeasurement
  :super ros::object
  :slots (_ssid _bssid _rssi _stamp ))

(defmethod wifi_scanner::WifiMeasurement
  (:init
   (&key
    ((:ssid __ssid) "")
    ((:bssid __bssid) "")
    ((:rssi __rssi) 0.0)
    ((:stamp __stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _ssid (string __ssid))
   (setq _bssid (string __bssid))
   (setq _rssi (float __rssi))
   (setq _stamp __stamp)
   self)
  (:ssid
   (&optional __ssid)
   (if __ssid (setq _ssid __ssid)) _ssid)
  (:bssid
   (&optional __bssid)
   (if __bssid (setq _bssid __bssid)) _bssid)
  (:rssi
   (&optional __rssi)
   (if __rssi (setq _rssi __rssi)) _rssi)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:serialization-length
   ()
   (+
    ;; string _ssid
    4 (length _ssid)
    ;; string _bssid
    4 (length _bssid)
    ;; float32 _rssi
    4
    ;; time _stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ssid
       (write-long (length _ssid) s) (princ _ssid s)
     ;; string _bssid
       (write-long (length _bssid) s) (princ _bssid s)
     ;; float32 _rssi
       (sys::poke _rssi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ssid
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ssid (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _bssid
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bssid (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _rssi
     (setq _rssi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wifi_scanner::WifiMeasurement :md5sum-) "50748349ed213168558a6fc822d7f7f0")
(setf (get wifi_scanner::WifiMeasurement :datatype-) "wifi_scanner/WifiMeasurement")
(setf (get wifi_scanner::WifiMeasurement :definition-)
      "string ssid
string bssid
float32 rssi
time stamp

")



(provide :wifi_scanner/WifiMeasurement "50748349ed213168558a6fc822d7f7f0")


