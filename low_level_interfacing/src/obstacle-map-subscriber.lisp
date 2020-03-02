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

    (setf *indexs* (floor
                (+
                 (*
                  (/
                  (- (cl-tf::y vect3) y)resolution) width)
                 (/ (- (cl-tf::x vect3) x)resolution))))
    (if (> (length data) *indexs*) (aref data *indexs*) 100 )
))

(defun robot-in-obstacle-stamped (point) 
     (robot-in-obstacle (cl-tf::origin point))
)

(defun robot-in-obstacle (point) 
     (circle-is-in-obstacle point 0.23 8)
)

;;Berechnet ob der Kreis mit dem radius von der obstaclemap gestört wird
(defun circle-is-in-obstacle (point radius amountPointOnCircle) 

	(setf *radians* (mapcar (lambda (listelem) (* (/ 6.28319 amountPointOnCircle) listelem)) 
		                         (loop :for n :from 1 :below amountPointOnCircle :collect n)))


	(setf *obstacleValues* (mapcar (lambda (listelem) 
		                (getMapPoint (cl-tf::make-3d-vector 
                                  (+ (* radius (cos listelem)) (cl-tf::x point)) 
                                  (+ (* radius (sin listelem)) (cl-tf::y point))
                                  0))
		        ) 
	*radians*))
         
        (= 0 (+ (apply '+ *obstacleValues* )  (getMapPoint point) ))


)

