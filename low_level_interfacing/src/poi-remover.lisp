(in-package :llif)

;;@author Philipp Klein
(defun poi-remover (point radius)
  "Receive point `point' and radius `radius'. Notify navigation to ignore `point' with `radius'."
  (roslisp:publish
   (roslisp:advertise "object_finder/ignore_area" 'navigation_msgs-msg:area)
   (roslisp:make-msg 'navigation_msgs-msg:area
                     :radius radius
                     :center (cl-tf::to-msg point)))
  (sleep 2))


