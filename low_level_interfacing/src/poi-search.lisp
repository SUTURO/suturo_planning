(in-package :llif)


(defparameter *searchMap* Nil)

;;@author Philipp Klein
(defun init-search-map ()
  "subscribes to the map topic and call the callback function save-init-map"
  (subscribe "map" "nav_msgs/OccupancyGrid" #'save-init-map))

;;@author Philipp Klein
(defun save-init-map (mapMsg)
  "saves the given msg in the searchMap parameter
  `mapMsg' the message to be saved"
  (if (not *searchMap*)
      (roslisp:ros-info (poi-search) "Map initialised")
      (defparameter *searchMap* (roslisp:modify-message-copy mapMsg))))

;;@author Philipp Klein
(defun mark-position-visited (stamped-pose radius)
  "marks a specific area in the map as already searched"
  ;;TODO)

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
        (if (and (/= i 0) (/= i (+ width 1)) (= (aref data i) 212)) ;;212 replace with correct value
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
      (cl-tf:make-3d-vector (+ x (* resolution col)) (+ y (* resolution row)) 0) (* resolution *max-size*))))
                   
