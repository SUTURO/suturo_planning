(in-package :llif)


(defparameter *currentObstacleMap* Nil)

;;@author Philipp Klein
(defun obstacle-map-listener ()
  "subscribes to the dynamic_obstacle_map topic and call the callback function save-obstacle-map"
  (subscribe "dynamic_obstacle_map" "nav_msgs/OccupancyGrid" #'save-obstacle-map)
  (roslisp:ros-info (obstacle-map-subscriber) "Map Subscribed"))

;;@author Philipp Klein
(defun save-obstacle-map (mapMsg)
  "saves the given msg in the currentObstacleMap parameter"
  (defparameter *currentObstacleMap* (roslisp:modify-message-copy mapMsg)))

;;@author Philipp Klein
(defun get-map-point (vect3)
  "returns the obstacle value of the given point"
  (roslisp:with-fields (
                        data
                        (resolution(resolution info))
                        (width(width info))
                        (x (x position origin info))
                        (y (y position origin info)))
      *currentObstacleMap*
    (setf *indexs* 
          (+
           (*
            (round
             (/
              (-
               (cl-tf::y vect3) y)
              resolution))
            width)
           (round
            (/
             (-
              (cl-tf::x vect3) x)
             resolution))))
    (if (and
         (>
          (length data) *indexs*)
         (> *indexs* 0))
        (aref data *indexs*) 111)))

;;@author Philipp Klein
(defun robot-in-obstacle-stamped (point)
  "wrappen for the robot-in-obstacle function to use a stamped pose"
  (robot-in-obstacle (cl-tf::origin point)))

;;@author Philipp Klein
(defun robot-in-obstacle (point)
  "returns if the obstacle value for the given point is lower then 66"
  (circle-is-in-obstacle point))

;;@author Philipp Klein
(defun circle-is-in-obstacle (point)
  "returns if the obstacle value for the given point is lower then 66"
  (< (get-map-point point) 66 ))




