
(cl:in-package :asdf)

(defsystem "mission_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Bounding_box" :depends-on ("_package_Bounding_box"))
    (:file "_package_Bounding_box" :depends-on ("_package"))
    (:file "class_pub" :depends-on ("_package_class_pub"))
    (:file "_package_class_pub" :depends-on ("_package"))
  ))