
(cl:in-package :asdf)

(defsystem "wifi_scanner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WifiMeasurement" :depends-on ("_package_WifiMeasurement"))
    (:file "_package_WifiMeasurement" :depends-on ("_package"))
  ))