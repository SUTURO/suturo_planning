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
  "returns a position of the center in the bigest area not searched yet"
  ;;TODO)





