(in-package :llif)

(defparameter *poi* (list ()))

;;@author Philipp Klein
(defun closest-poi (point)
  "returns the closest poi in relation to the given point
  `point' the point which is taken as source for the distance measurement"
  (closest-point-in-list point *poi*))

;;@author Philipp Klein
(defun closest-point-in-list (point stampedList)
	"returns the closest point in relation of the given point
  `point' the point which is taken as source for the distance measurement
  `stampedList' the list of points to be sorted"
  (cond
    ((null stampedList) nil)
    ((null (rest stampedList)) (first stampedList))
    ((null (first stampedList)) (closestPointInList point (rest stampedList)))
    ((< 
      (cl-tf::v-dist (cl-tf::origin point)
                     (cl-tf::origin (first stampedList))) 
      (cl-tf::v-dist (cl-tf::origin point)
                     (cl-tf::origin (second stampedList))))
     (closestPointInList point (cons (first stampedList) 
                                     (rest (rest stampedList)))))
    (t (closestPointInList point (rest stampedList))))) 

;;@author Philipp Klein
(defun sorted-poi-by-distance (point)
  "sort the poi list by the distance to the given point
  `point' the point which is taken as source for the distance measurement"
  (sorted-stamped-by-distance point *poi*))

;;@author Philipp Klein
(defun sorted-stamped-by-distance (point list)
  "sort a list stamped poses by the distance to the given point
  `point' the point which is taken as source for the distance measurement
  `list' the list of points to be sorted"
  (sort (copy-list (remove-if #'null list))
        (lambda (ers zwei) 
          (< 
           (cl-tf::v-dist point
                          (cl-tf::origin ers)) 
           (cl-tf::v-dist point
                          (cl-tf::origin zwei))) )))

;;@author Philipp Klein
(defun add-stamped-poi (stamped)
  "add a stamped pose to the list of pois
  `stamped' the point to be added"
	(defparameter *poi* (append *poi* (list stamped))))

;;@author Philipp Klein
(defun point-listener ()
  "subscribe to bject_finder and call the callback function add-poi-from-topic"
  (subscribe "/object_finder/object_finder" "geometry_msgs/PoseArray" #'add-poi-from-topic)
  (roslisp:ros-info (poi-subscriber) "POI Subscriber started"))

;;@author Philipp Klein
(defun add-poi-from-topic (poseArrayMsg)
  "save the given points to the parameter poi
  `poseArrayMsg' the message to be saved"
  (llif::mark-position-visited 1)
  (llif::publish-debug-search-map)
  (defparameter *poi* (list()))
  (roslisp:with-fields (poses) poseArrayMsg
    (mapcar 
     (lambda (arg) 
       (add-stamped-poi
        (cl-tf::pose->pose-stamped "map" 0.0 (cl-tf::from-msg arg)))
       )(coerce poses 'list))))
