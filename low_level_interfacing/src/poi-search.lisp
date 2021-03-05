(in-package :llif)


(defparameter *searchMap* Nil)

;;@author Philipp Klein
(defun init-search-map ()
  "subscribes to the map topic and call the callback function save-init-map"
  (subscribe "static_distance_map_ref" "nav_msgs/OccupancyGrid" #'save-init-map)
  (roslisp:ros-info (poi-search-map) "Map Subscribed"))

;;@author Philipp Klein
(defun save-init-map (mapMsg)
  "saves the given msg in the searchMap parameter
  `mapMsg' the message to be saved"
  (if (not *searchMap*)
      (defparameter *searchMap* (roslisp:modify-message-copy mapMsg))))


;;@author Philipp Klein
(defun publish-debug-search-map ()
  "publishes the current searched map"
   (roslisp:publish (advertise "search_map" "nav_msgs/OccupancyGrid") *searchMap*)
  )

;;@author Philipp Klein
(defun mark-position-visited (radius)
  "marks a specific area in the map as already searched
  `radius' the radius around the robot that should be marked as searched"
  (roslisp:with-fields
      (data
      (resolution(resolution info))
      (width(width info))
      (x (x position origin info))
      (y (y position origin info)))
  *searchMap*
    (setf *position* (cl-tf::transform-stamped->pose-stamped
          (cl-tf::lookup-transform
           cram-tf::*transformer*
           "map" "base_footprint")))
    (setf *vect3* (cl-tf:origin *position*))
    (setf *indexs* 
          (+
           (*
            (round
             (/
              (-
               (cl-tf::y *vect3*) y)
              resolution))
            width)
           (round
            (/
             (-
              (cl-tf::x *vect3*) x)
             resolution))))
    (setf *rradius* (round (* radius resolution)))
    (if (and
         (>
          (length data) *indexs*)
         (> *indexs* 0))
        
        (multiple-value-bind (row col) (floor *indexs* width)
        (loop for i from (- row *rradius*) to (+ row *rradius*) do
          (loop for j from (- col *rradius*) to (+ col *rradius*) do
            (setf (aref data (+(* row width) col)) 66)
         )
         )))
  ))

;;@author Philipp Klein
(defun find-biggest-notsearched-space ()
  "returns the position of the bottom left corner of the bigest area not searched yet and the size"
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
    (loop for i from 0 to (- (length data) 1) do
      ;;(multiple-value-bind (row col) (floor i width)
        (if (and (>= i (+ width 1)) (= (aref data i) 0))
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
      (print *bl-coord*)
      (print *realsize*)
      (publish-debug-square (list *bl-coord*
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector 0 *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* 0 0)))))))

;;@author Philipp Klein
(defun publish-debug-square (pose-list)
  "publishs a square marker, to visualise regions in rviz
  `pose-list' the 4 corners of the square as a 3d Vector"
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
