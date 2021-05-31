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
(defun create-map-message (map-message data-array)
  "publishes the current searched map"
  (roslisp:with-fields (header info) map-message
    (roslisp::make-message "nav_msgs/OccupancyGrid"
                           (header) header
                           (info) info
                           (data) data-array)))

;;@author Philipp Klein
(defun publish-debug-search-map ()
  "publishes the current searched map"
   (roslisp:publish (advertise "search_map" "nav_msgs/OccupancyGrid") *searchMap*)
  )

;;@author Philipp Klein
(defun mark-position-visited (radius &optional position)
  "marks a specific area in the map as already searched
  `radius' the radius around the robot that should be marked as searched"
  (roslisp:with-fields
      ((datas (data))
      (resolution(resolution info))
      (width(width info))
      (x (x position origin info))
      (y (y position origin info)))
      *searchMap*
    (if position
         (setf *vect3* (cl-tf:origin position))
         (setf *vect3* (roslisp::with-fields (translation)
                            (cl-tf::lookup-transform
                            cram-tf::*transformer*
                            "map"
                            "base_footprint")
                        translation)))
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
    ;;(print *position*)
    ;;(print *vect3*)
    (setf *rradius* (round (/ radius resolution)))
    (if (and
         (>
          (length datas) *indexs*)
         (> *indexs* 0))
        
        (multiple-value-bind (row col) (floor *indexs* width)
        (loop for i from (- row *rradius*) to (+ row *rradius*) do
          (loop for j from (- col *rradius*) to (+ col *rradius*) do
            (if (< (+(* i width) j) (length datas))
            (setf (aref datas (+(* i width) j)) 66))
         )
              )))
    (defparameter *searchMap* (create-map-message *searchMap* datas))
  ))

;;@author Philipp Klein
(defun find-biggest-notsearched-space (&optional debug)
  "returns the position of the bottom left corner of the bigest area not searched yet and the size"
  (print "searching empty square")
  (roslisp:with-fields (data
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
      ;;(print i)
      (if (and
           (>= i width)
           (not (= 0 (mod i width)))
           (= (aref data i) 0))
            (progn (setf (aref *copy* i)
                  (+  
                   (min (aref *copy* (- i 1))
                       (aref *copy* (- i width))
                       (aref *copy* (- i width 1)))
                   1))
            (if (< *max-size* (aref *copy* i))
              (progn
                (setf *max-size* (aref *copy* i))
                (setf *bl-corner* i))))
            (setf (aref *copy* i) 0)))
    (roslisp:publish (advertise "search_map_algo" "nav_msgs/OccupancyGrid") (roslisp:modify-message-copy *searchMap* (data) *copy*))
    (multiple-value-bind (row col) (floor *bl-corner* width) ;;floor or round
      (setf *bl-coord*
            (cl-tf:make-3d-vector
             (+ x (* resolution col))
             (+ y (* resolution row))
             0))
      (setf *realsize*  (* *max-size* resolution))
      (publish-debug-square (list *bl-coord*
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector 0 *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* *realsize* 0))
                                  (cl-tf:v- *bl-coord* (cl-tf:make-3d-vector *realsize* 0 0))))
      (setf *position* (cl-tf::transform->pose
        (cl-tf::lookup-transform
          cram-tf::*transformer*
          "map" "base_footprint")))
      (let ((center (cl-tf:make-pose-stamped "map" 0 (cl-tf:v- *bl-coord*
          (cl-tf:make-3d-vector (/ *realsize* 2) (/ *realsize* 2) 0))
             (cl-tf::make-quaternion 0 0 0 1))))
      (if
       (and
        (llif::global-planner-reachable *position* center)
        (not (llif::prolog-is-pose-outside
         (cl-tf::x center)
         (cl-tf::y center)
         (cl-tf::z center))))
       ;;TODO maybe change it from middle to smth else
       center
       (progn
         (mark-position-visited (/ *realsize* 2) center)
         (find-biggest-notsearched-space)
         (if debug (sleep 0.5))
         ))
      ))))

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
