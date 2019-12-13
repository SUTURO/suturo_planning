(in-package : comp)

;;; TF ;;;
(defparameter *tf-listener* nil)

(defun get-tf-listener ()
  (unless *tf-listener*
    (setf *tf-listener* (make-instance 'cl-tf2:buffer-client))
    (handler-case
     (cl-tf2:lookup-transform *tf-listener* "map" "odom" :timeout 3)
      (CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED-ERROR () (roslisp:ros-warn (get-tf-listener) "tf-listener takes longer than 20 seconds to get odom in map."))
      (CL-TRANSFORMS-STAMPED:TIMEOUT-ERROR
       () (roslisp:ros-warn (get-tf-listener) "tf-listener takes longer than 20 seconds to get odom in map."))))
  *tf-listener*)
