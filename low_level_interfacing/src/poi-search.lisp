(in-package :llif)


(defparameter *searchMap* Nil)

;;@author Philipp Klein
(defun init-search-map ()
  "subscribes to the map topic and call the callback function save-init-map"
  (subscribe "map" "nav_msgs/OccupancyGrid" #'save-init-map)
  (roslisp:ros-info (poi-search-map) "Map Subscribed")))

;;@author Philipp Klein
(defun save-init-map (mapMsg)
  "saves the given msg in the searchMap parameter
  `mapMsg' the message to be saved"
  (if (not *searchMap*)
      (roslisp:ros-info (poi-search-map) "Map updated")
      (defparameter *searchMap* (roslisp:modify-message-copy mapMsg))))

;;@author Philipp Klein
(defun mark-position-visited (stamped-pose radius)
  "marks a specific area in the map as already searched"
  ;;TODO
  )

;;@author Philipp Klein
(defun find-biggest-notsearched-space ()
  "returns the position of the bottom left corner of the bigest area not searched yet and the size"
  ;;TODO
  (roslisp:with-fields (
                        data
                        (resolution(resolution info))
                        (width (width info))
                        (x (x position origin info))
                        (y (y position origin info)))
      *searchMap*
    (setf *max-size* 0)
    (setf *bl-corner* Nil)
    (setf *copy* (make-array (length data) :initial-element 0))
    (loop for i from 0 to (length data) do
      ;;(multiple-value-bind (row col) (floor i width)
        (if (and (/= i 0) (/= i (+ width 1)) (= (aref data i) 0)) ;;212 replace with correct value
            (setf (aref *copy* i)
                  (+  
                   (min (aref *copy* (- i 1))
                       (aref *copy* (- i width))
                       (aref *copy* (- i width 1)))
                   1)))
        (if (< *max-size* (aref *copy* i))
            (setf *max-size* (aref *copy* i))
            (setf *bl-corner* i)))
    (multiple-value-bind (row col) (floor *bl-corner* width) ;;floor or round
      (setf *bl-coord*
            (cl-tf:make-3d-vector
             (+ x (* resolution col))
             (+ y (* resolution row))
             0))
      (setf *realsize*  (* resolution *max-size*))
      (publish-debug-square (list *bl-coord*
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector 0 *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* 0 0)))))))

;;@author Philipp Klein
(defun publish-debug-square (pose-list)
  (nconc pose-list (list (car pose-list)))
  (setf pose-list (mapcar #'cl-tf:make-point-msg pose-list))
  (setf point-array (make-array 5 :initial-contents pose-list))
  (roslisp:publish (advertise "debug_square" "visualization_msgs/Marker")
                   (roslisp:make-msg
                     "visualization_msgs/Marker"
                    (frame_id header) "map"
                    (stamp header) (ros-time)
                    (ns) "poi-search"
                    (id) 0
                    (type) 4
                    (action) 0
                    (x position pose) 0
                    (y position pose) 0
                    (z position pose) 0
                    (x orientation pose) 0
                    (y orientation pose) 0
                    (z orientation pose) 0
                    (w orientation pose) 1
                    (x scale) 0.05
                    (a color) 1
                    (r color) 0
                    (g color) 1
                    (b color) 0
                    (points) (make-array 5 :initial-contents pose-list))))
