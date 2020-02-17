(in-package :llif)

(defun make-poi (px py pz ox oy oz ow)
		(cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow))))

(defparameter *poi* (list
        ()))

(defun popPoi () 
     (pop *poi*))

(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi* (list(make-poi px py pz ox oy oz ow))))
        ;;(append *poi* (make-poi px py pz ox oy oz ow))
)

(defun add-stamped-poi (stamped)
	(defparameter *poi* (append *poi* (list(stamped))))
)



(defun poi-listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "object_finder" "geometry_msgs/PoseArray" #'addPoiFromTopic)))

(defun addPoiFromTopic (poseArrayMsg) 
        (roslisp:ros-info (poi-subscriber) "added POIs to List")
        (mapcar 'add-stamped-poi poseArrayMsg)
)
