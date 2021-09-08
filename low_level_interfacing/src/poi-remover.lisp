(in-package :llif)

;;@author Philipp Klein
(defun poi-remover (point radius)
  "notify navigation to ignore a point `point' with a radius `radius'"
  (roslisp:publish
   (roslisp:advertise "object_finder/ignore_area" 'navigation_msgs-msg:area)
   (roslisp:make-msg 'navigation_msgs-msg:area
                        :radius radius
                        :center (cl-tf::to-msg point)))
  (sleep 1))


