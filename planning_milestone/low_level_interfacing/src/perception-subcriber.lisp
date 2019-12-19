(in-package :llif)

(defvar *color* 'red)

(defun listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "/perception_output/planning"  "perception_msgs/RSObject" #'sorting)))

(defun sorting(msg)
 (defvar *px* (roslisp:msg-slot-value msg :x))
 (roslisp:ros-info (init-clients)"*px*")
 (defvar *py* (roslisp:msg-slot-value msg :y))
 (defvar *pz* (roslisp:msg-slot-value msg :z))
 (defvar *pw* (roslisp:msg-slot-value msg :w))
 (defvar *ph* (roslisp:msg-slot-value msg :h))
 (defvar *pd* (roslisp:msg-slot-value msg :d))
 (defvar *confidence* (roslisp:msg-slot-value msg :confidence))
 (defvar *color* (roslisp:msg-slot-value msg :color_name))

)
