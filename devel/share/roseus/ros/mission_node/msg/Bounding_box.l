;; Auto-generated. Do not edit!


(when (boundp 'mission_node::Bounding_box)
  (if (not (find-package "MISSION_NODE"))
    (make-package "MISSION_NODE"))
  (shadow 'Bounding_box (find-package "MISSION_NODE")))
(unless (find-package "MISSION_NODE::BOUNDING_BOX")
  (make-package "MISSION_NODE::BOUNDING_BOX"))

(in-package "ROS")
;;//! \htmlinclude Bounding_box.msg.html


(defclass mission_node::Bounding_box
  :super ros::object
  :slots (_detected_class _cx _cy ))

(defmethod mission_node::Bounding_box
  (:init
   (&key
    ((:detected_class __detected_class) "")
    ((:cx __cx) 0)
    ((:cy __cy) 0)
    )
   (send-super :init)
   (setq _detected_class (string __detected_class))
   (setq _cx (round __cx))
   (setq _cy (round __cy))
   self)
  (:detected_class
   (&optional __detected_class)
   (if __detected_class (setq _detected_class __detected_class)) _detected_class)
  (:cx
   (&optional __cx)
   (if __cx (setq _cx __cx)) _cx)
  (:cy
   (&optional __cy)
   (if __cy (setq _cy __cy)) _cy)
  (:serialization-length
   ()
   (+
    ;; string _detected_class
    4 (length _detected_class)
    ;; int16 _cx
    2
    ;; int16 _cy
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _detected_class
       (write-long (length _detected_class) s) (princ _detected_class s)
     ;; int16 _cx
       (write-word _cx s)
     ;; int16 _cy
       (write-word _cy s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _detected_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _detected_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _cx
     (setq _cx (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _cy
     (setq _cy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mission_node::Bounding_box :md5sum-) "899615dfe4b53d2fa5fba3dfbb98944a")
(setf (get mission_node::Bounding_box :datatype-) "mission_node/Bounding_box")
(setf (get mission_node::Bounding_box :definition-)
      "string detected_class
int16 cx
int16 cy

")



(provide :mission_node/Bounding_box "899615dfe4b53d2fa5fba3dfbb98944a")


