; Auto-generated. Do not edit!


(cl:in-package mission_node-msg)


;//! \htmlinclude class_pub.msg.html

(cl:defclass <class_pub> (roslisp-msg-protocol:ros-message)
  ((classify_class
    :reader classify_class
    :initarg :classify_class
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:fixnum
    :initform 0))
)

(cl:defclass class_pub (<class_pub>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <class_pub>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'class_pub)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_node-msg:<class_pub> is deprecated: use mission_node-msg:class_pub instead.")))

(cl:ensure-generic-function 'classify_class-val :lambda-list '(m))
(cl:defmethod classify_class-val ((m <class_pub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_node-msg:classify_class-val is deprecated.  Use mission_node-msg:classify_class instead.")
  (classify_class m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <class_pub>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_node-msg:confidence-val is deprecated.  Use mission_node-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <class_pub>) ostream)
  "Serializes a message object of type '<class_pub>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'classify_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'classify_class))
  (cl:let* ((signed (cl:slot-value msg 'confidence)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <class_pub>) istream)
  "Deserializes a message object of type '<class_pub>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'classify_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'classify_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'confidence) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<class_pub>)))
  "Returns string type for a message object of type '<class_pub>"
  "mission_node/class_pub")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'class_pub)))
  "Returns string type for a message object of type 'class_pub"
  "mission_node/class_pub")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<class_pub>)))
  "Returns md5sum for a message object of type '<class_pub>"
  "236eed931e14e0bcfce3c6d0f10d08fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'class_pub)))
  "Returns md5sum for a message object of type 'class_pub"
  "236eed931e14e0bcfce3c6d0f10d08fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<class_pub>)))
  "Returns full string definition for message of type '<class_pub>"
  (cl:format cl:nil "string classify_class~%int16 confidence~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'class_pub)))
  "Returns full string definition for message of type 'class_pub"
  (cl:format cl:nil "string classify_class~%int16 confidence~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <class_pub>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'classify_class))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <class_pub>))
  "Converts a ROS message object to a list"
  (cl:list 'class_pub
    (cl:cons ':classify_class (classify_class msg))
    (cl:cons ':confidence (confidence msg))
))
