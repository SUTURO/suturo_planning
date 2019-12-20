(in-package :llif)

(defvar *color* "red")

(defun listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "/perception_output/planning"  "perception_msgs/RSObject" #'sorting)))

(defun sorting(msg)
  (let ((*px* (roslisp:msg-slot-value msg :x))
        (*py* (roslisp:msg-slot-value msg :y))
        (*pz* (roslisp:msg-slot-value msg :z))
        (*pw* (roslisp:msg-slot-value msg :w))
        (*ph* (roslisp:msg-slot-value msg :h))
        (*pd* (roslisp:msg-slot-value msg :d))
        (*confidence* (roslisp:msg-slot-value msg :confidence))
        (*color-of-object* (roslisp:msg-slot-value msg :color_name)))

    (if (equalp *color-of-object* *color*)
        (call-grasp-action *px* *py* 0.28 0.0 0.7 0.0 0.7 *pw* *ph* *pd*)
)))
