(in-package :llif)

(defun make-point (px py pz ox oy oz ow)
		(cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow))))
;;defparemeter once on the top of the file 
(defparameter *poi* (list ()))


(defun closestPoi (point)
  (closestPointInList point *poi*))

(defun closestPointInList (point stampedList)
	"returns the closest Poi in relation of the given point"
	
 (cond
       ((null stampedList) nil)
	;;indent region.. control + alt + \ i think 
       ((null (rest stampedList)) (first stampedList))
	;;leerzeilen nur bei gedankenspruengen..
       ((null (first stampedList)) (closestPointInList point (rest stampedList)))

       ((< 
          (cl-tf::v-dist (cl-tf::origin point) ;;next line
			 (cl-tf::origin (first stampedList))) 
          (cl-tf::v-dist (cl-tf::origin point) (cl-tf::origin (second stampedList)))
        )
        (closestPointInList point (cons (first stampedList) 
                       (rest (rest stampedList)))))
       (t (closestPointInList point (rest stampedList))))) 


(defun sortedPoiByDistance (point) 
   (sort (copy-list *poi*)
         (lambda (ers zwei) 
                 (< 
	           (cl-tf::v-dist (cl-tf::origin point)
			 (cl-tf::origin ers)) 
                   (cl-tf::v-dist (cl-tf::origin point) 
                         (cl-tf::origin zwei))) )))


(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi*  ;;next line
				    (list(make-point px py pz ox oy oz ow))))
        ;;(append *poi* (make-point px py pz ox oy oz ow))
)

(defun add-stamped-poi (stamped)
	(defparameter *poi* (append *poi* (list stamped)))
)



(defun point-listener ()
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
