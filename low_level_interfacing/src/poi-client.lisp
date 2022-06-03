(in-package :llif)

(defparameter *poi* (list ()))

;;@author Philipp Klein
(defun closest-poi (point)
  "Receives point `point'. Returns the closest poi in relation to the given `point'."
  (closest-point-in-list point *poi*))

;;@author Philipp Klein
(defun closest-point-in-list (point stampedList)
  "Receives point `point' and list `stampedList'. Returns the closest point in `stampedList' in relation of `point'."
  (cond
    ((null stampedList) nil)
    ((null (rest stampedList)) (first stampedList))
    ((null (first stampedList)) (closest-point-in-list point (rest stampedList)))
    ((< 
      (cl-tf::v-dist (cl-tf::origin point)
                     (cl-tf::origin (first stampedList))) 
      (cl-tf::v-dist (cl-tf::origin point)
                     (cl-tf::origin (second stampedList))))
     (closest-point-in-list point (cons (first stampedList) 
                                        (rest (rest stampedList)))))
    (t (closest-point-in-list point (rest stampedList))))) 

;;@author Philipp Klein
(defun sorted-poi-by-distance (point)
  "Receives point `point'. Sort the poi list by the distance to `point'."
  (sorted-stamped-by-distance point *poi*))

;;@author Philipp Klein
(defun sorted-stamped-by-distance (point list)
  "Receives point `point' and list `list'. Sort `list' by the distance to `point'."
  (sort (copy-list (remove-if #'null list))
        (lambda (ers zwei) 
          (< 
           (cl-tf::v-dist point
                          (cl-tf::origin ers)) 
           (cl-tf::v-dist point
                          (cl-tf::origin zwei))))))

;;@author Philipp Klein
(defun add-stamped-poi (stamped)
  "Receives stamped pose `stamped'. Add `stamped' to the list of pois."
  (defparameter *poi* (append *poi* (list stamped))))

;;@author Philipp Klein
(defun point-listener ()
 "Subscribe to object\_finder and call the callback function add-poi-from-topic."
  (subscribe "/object_finder/object_finder" "geometry_msgs/PoseArray" #'add-poi-from-topic)
  (roslisp:ros-info (poi-subscriber)
                    "POI Subscriber started"))

;;@author Philipp Klein
(defun add-poi-from-topic (poseArrayMsg)
  "Receives message  `poseArrayMsg'. Save `poseArrayMsg' to the parameter poi."
  (llif::mark-position-visited 0.7)
  (llif::publish-debug-search-map)
  (defparameter *poi* (list()))
  (roslisp:with-fields (poses) poseArrayMsg
    (mapcar 
     (lambda (arg) 
       (add-stamped-poi
        (cl-tf::pose->pose-stamped
         "map" 0.0
         (cl-tf::from-msg arg))))
     (coerce poses 'list))))
