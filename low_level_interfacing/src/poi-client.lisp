(in-package :llif)

(defun make-poi (px py pz ox oy oz ow)
		(cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow))))

(defparameter *poi* (list ()))


(defun closestPoi (point)
  (closestPointInList point *poi*))

(defun closestPointInList (point stampedList)
 (cond
       ((null stampedList) nil)

       ((null (rest stampedList)) (first stampedList))

       ((null (first stampedList)) (closestPointInList point (rest stampedList)))

       ((< 
          (cl-tf::v-dist (cl-tf::origin point) (cl-tf::origin (first stampedList))) 
          (cl-tf::v-dist (cl-tf::origin point) (cl-tf::origin (second stampedList)))
        )
        (closestPointInList point (cons (first stampedList) 
                       (rest (rest stampedList)))))
       (t (closestPointInList point (rest stampedList))))) 

(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi* (list(make-poi px py pz ox oy oz ow))))
        ;;(append *poi* (make-poi px py pz ox oy oz ow))
)

(defun add-stamped-poi (stamped)
	(defparameter *poi* (append *poi* (list stamped)))
)



(defun poi-listener ()
  (subscribe "object_finder" "geometry_msgs/PoseArray" #'addPoiFromTopic)
  ;;(with-ros-node ("listener" :spin t)
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
  ;;(roslisp:ros-info (poi-subscriber) "closest point to zero? xD ~a" (closestPoi
  ;;(cl-tf::make-pose-stamped "map" 0.0 (cl-tf::make-3d-vector 0 0 0) (cl-tf::make-quaternion 0 0.7 0 0.7) )))
)
