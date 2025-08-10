; Auto-generated. Do not edit!


(cl:in-package mission_node-msg)


;//! \htmlinclude Bounding_box.msg.html

(cl:defclass <Bounding_box> (roslisp-msg-protocol:ros-message)
  ((detected_class
    :reader detected_class
    :initarg :detected_class
    :type cl:string
    :initform "")
   (cx
    :reader cx
    :initarg :cx
    :type cl:fixnum
    :initform 0)
   (cy
    :reader cy
    :initarg :cy
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Bounding_box (<Bounding_box>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bounding_box>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bounding_box)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_node-msg:<Bounding_box> is deprecated: use mission_node-msg:Bounding_box instead.")))

(cl:ensure-generic-function 'detected_class-val :lambda-list '(m))
(cl:defmethod detected_class-val ((m <Bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_node-msg:detected_class-val is deprecated.  Use mission_node-msg:detected_class instead.")
  (detected_class m))

(cl:ensure-generic-function 'cx-val :lambda-list '(m))
(cl:defmethod cx-val ((m <Bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_node-msg:cx-val is deprecated.  Use mission_node-msg:cx instead.")
  (cx m))

(cl:ensure-generic-function 'cy-val :lambda-list '(m))
(cl:defmethod cy-val ((m <Bounding_box>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_node-msg:cy-val is deprecated.  Use mission_node-msg:cy instead.")
  (cy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bounding_box>) ostream)
  "Serializes a message object of type '<Bounding_box>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'detected_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'detected_class))
  (cl:let* ((signed (cl:slot-value msg 'cx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bounding_box>) istream)
  "Deserializes a message object of type '<Bounding_box>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'detected_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'detected_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cy) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bounding_box>)))
  "Returns string type for a message object of type '<Bounding_box>"
  "mission_node/Bounding_box")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bounding_box)))
  "Returns string type for a message object of type 'Bounding_box"
  "mission_node/Bounding_box")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bounding_box>)))
  "Returns md5sum for a message object of type '<Bounding_box>"
  "899615dfe4b53d2fa5fba3dfbb98944a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bounding_box)))
  "Returns md5sum for a message object of type 'Bounding_box"
  "899615dfe4b53d2fa5fba3dfbb98944a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bounding_box>)))
  "Returns full string definition for message of type '<Bounding_box>"
  (cl:format cl:nil "string detected_class~%int16 cx~%int16 cy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bounding_box)))
  "Returns full string definition for message of type 'Bounding_box"
  (cl:format cl:nil "string detected_class~%int16 cx~%int16 cy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bounding_box>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'detected_class))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bounding_box>))
  "Converts a ROS message object to a list"
  (cl:list 'Bounding_box
    (cl:cons ':detected_class (detected_class msg))
    (cl:cons ':cx (cx msg))
    (cl:cons ':cy (cy msg))
))
