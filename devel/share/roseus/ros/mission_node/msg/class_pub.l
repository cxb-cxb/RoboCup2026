;; Auto-generated. Do not edit!


(when (boundp 'mission_node::class_pub)
  (if (not (find-package "MISSION_NODE"))
    (make-package "MISSION_NODE"))
  (shadow 'class_pub (find-package "MISSION_NODE")))
(unless (find-package "MISSION_NODE::CLASS_PUB")
  (make-package "MISSION_NODE::CLASS_PUB"))

(in-package "ROS")
;;//! \htmlinclude class_pub.msg.html


(defclass mission_node::class_pub
  :super ros::object
  :slots (_classify_class _confidence ))

(defmethod mission_node::class_pub
  (:init
   (&key
    ((:classify_class __classify_class) "")
    ((:confidence __confidence) 0)
    )
   (send-super :init)
   (setq _classify_class (string __classify_class))
   (setq _confidence (round __confidence))
   self)
  (:classify_class
   (&optional __classify_class)
   (if __classify_class (setq _classify_class __classify_class)) _classify_class)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; string _classify_class
    4 (length _classify_class)
    ;; int16 _confidence
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _classify_class
       (write-long (length _classify_class) s) (princ _classify_class s)
     ;; int16 _confidence
       (write-word _confidence s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _classify_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _classify_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _confidence
     (setq _confidence (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mission_node::class_pub :md5sum-) "236eed931e14e0bcfce3c6d0f10d08fb")
(setf (get mission_node::class_pub :datatype-) "mission_node/class_pub")
(setf (get mission_node::class_pub :definition-)
      "string classify_class
int16 confidence
")



(provide :mission_node/class_pub "236eed931e14e0bcfce3c6d0f10d08fb")


