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

;; Mit BulletWorld eine geeignete Position neben dem punkt finden
;;(defun nearestPoi (point) 
     ;;(mapcar 
     ;;    (lambda (arg) 
     ;;        (list (cl-tf::v-dist (cl-tf::origin point) (cl-tf::origin arg)) (arg))
     ;;)*poi*))

(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi* (list(make-poi px py pz ox oy oz ow))))
        ;;(append *poi* (make-poi px py pz ox oy oz ow))
)

(defun add-stamped-poi (stamped)
	(defparameter *poi* (append *poi* (list stamped)))
)



(defun poi-listener ()
  (with-ros-node ("listener" :spin t)
    (subscribe "object_finder" "geometry_msgs/PoseArray" #'addPoiFromTopic))
  (roslisp:ros-info (poi-subscriber) "POI Subscriber started")
)

(defun addPoiFromTopic (poseArrayMsg) 
  ;;(roslisp:ros-info (poi-subscriber) "added POIs to List")
  (defparameter *poi* (list()))
    (roslisp:with-fields (poses) poseArrayMsg
        (mapcar 
            (lambda (arg) 
                         (add-stamped-poi(cl-tf::pose->pose-stamped "map" 0.0 (cl-tf::from-msg arg)))
                         ;;(roslisp:ros-info (poi-subscriber) "added ~a"arg)
                         ;;(roslisp:ros-info (poi-subscriber) "Die ganze Liste: ~a"*poi*)
            )(coerce poses 'list)
        )
    )
)
