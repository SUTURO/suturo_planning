(in-package :llif)


(defparameter *currentObstacleMap* Nil)


(defun obstacle-map-listener ()
  (subscribe "dynamic_obstacle_map_ref" "nav_msgs/OccupancyGrid" #'saveObstacleMap)
  ;;(with-ros-node ("listener" :spin t)
  (roslisp:ros-info (obstacle-map-subscriber) "Map Subscribed")
)

(defun saveObstacleMap (mapMsg) 
  (defparameter *currentObstacleMap* mapMsg)
)

(defun getMapPoint (vect3) 
  (roslisp:with-fields (
                        data
                        (resolution(resolution info))
                        (width(width info))
                        (x (x position origin info))
                        (y (y position origin info)))
      *currentObstacleMap*
    (aref data (floor
                (+
                 (*
                  (/
                  (- (cl-tf::y vect3) y)resolution) width)
                 (/ (- (cl-tf::x vect3) x)resolution)))))
   )

