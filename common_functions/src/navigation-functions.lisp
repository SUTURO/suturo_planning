(in-package :comf)
;;TODO: fix indentation similar to high-level/grocery-execute

(defparameter *currentOrigin* Nil)
(defparameter *goalOrigin* Nil)
(defparameter *newgoalOrigin* Nil)
(defparameter *orientation* Nil)
(defparameter *newgoalstamped* Nil)
(defparameter *radians* Nil)
(defparameter *alternatePositions* Nil)
(defparameter *polyCorner1* Nil)
(defparameter *isPoly* Nil)
(defparameter *removed* Nil)
(defvar *pose*)

;;author Philipp Klein
(defun points-around-point (distance point amountAlternatePositions turn)
  "return a given amount of points with a given distance around a list of points
  `distance' The distance, with which the points around the point are generated
  `point' The point from which the others are to be generated
  `amountAlternatePositions' The number of points to be generated
  `turn' whether the robot should be turned 90 degrees to the target at the end"
  (setf *currentOrigin*
        (cl-tf::origin
         (cl-tf::transform-stamped->pose-stamped
          (cl-tf::lookup-transform
           cram-tf::*transformer*
           "map" "base_footprint"))))
  (setf *goalOrigin* (cl-tf::origin point))
  (setf *newgoalOrigin* (cl-tf::v+ *goalOrigin* 
                                   (cl-tf::v* 
                                    (cl-tf::v- *currentOrigin* *goalOrigin*) 
                                    (/ distance
                                       (cl-tf::v-dist
                                        *currentOrigin*
                                        *goalOrigin*)))))
  (setf *radians*
        (mapcar (lambda
                    (listelem)
                  (*
                   (/
                    6.28319
                    amountAlternatePositions)
                   listelem)) 
                (loop :for n :from 1 :below amountAlternatePositions :collect n)))
	(setf *alternatePositions*
        (mapcar
         (lambda (listelem)
           (
            cl-tf:make-pose-stamped
            "map"
            (roslisp::ros-time)
            (cl-tf::make-3d-vector 
             (+
              (*
               distance
               (cos listelem))
              (cl-tf::x *goalOrigin*)) 
             (+
              (*
               distance
               (sin listelem))
              (cl-tf::y *goalOrigin*))
             0)
            (cl-tf:euler->quaternion
             :ax 0.0 :ay 0.0 :az
             (let*(    
                   (y
                     (-
                      (cl-tf::y *goalOrigin*)
                      (+
                       (*
                        distance
                        (sin listelem))
                       (cl-tf::y *goalOrigin*)))) 
                   (x
                     (-
                      (cl-tf::x *goalOrigin*)
                      (+
                       (*
                        distance
                        (cos listelem))
                       (cl-tf::x *goalOrigin*))))
                   (atanValue (atan (/ y x))))
               (if turn
                   (if (< x 0)
                       (-
                        atanValue 1.57)
                       (+
                        atanValue 1.57))
                   atanValue))))) 
         *radians*))
  (setf *alternatePositions*
        (llif::sorted-stamped-by-distance
         (roslisp::with-fields (translation)
             (cl-tf::lookup-transform
              cram-tf::*transformer*
              "map" "base_footprint")
           translation)
         *alternatePositions*)))

;;author Philipp Klein
(defun pose-with-distance-to-points (distance points number-alternate-positions turn)
  "move the roboter to a point with a given distance around the closest point
  `distance' The distance, with which the points around the point are generated
  `point' The point from which the others are to be generated
  `amountAlternatePositions' The number of points to be generated
  `turn' whether the robot should be turned 90 degrees to the target at the end"
  (let* ((positions
           (flatten 
            (mapcar 
             (lambda (elem) (remove-if #'null elem))
             (mapcar
              (lambda (elem) (remove-if-not #'llif::robot-in-obstacle-stamped elem)) 
              (mapcar 
               (lambda (elem) (points-around-point
                               distance elem  number-alternate-positions turn))
                          points))))))
      (publish-msg 
        (advertise "poi_positions" "geometry_msgs/PoseArray")
          :header
          (roslisp:make-msg "std_msgs/Header" (frame_id) "map" (stamp) (roslisp:ros-time))
          :poses
          (make-array (length positions)
                      :initial-contents
                      (mapcar #'cl-tf::to-msg
                              (mapcar #'cl-tf::pose-stamped->pose
                                      positions))))
    (roslisp::ros-info (navigation-functions) "Started Designator with positions ~a" positions)
    (let* ((?successfull-pose (try-movement-stampedList positions))
           (?desig (desig:a motion
                            (type going) 
                            (pose ?successfull-pose))))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)
               (e)
             (setf ?successfull-pose (try-movement-stampedList positions))
             (cpl:do-retry going-retry
               (print "Failed Going Designator")
               (roslisp:ros-warn (move-fail) "~%Failed to go to Point~%")
               (cpl:retry))))
        (exe:perform ?desig))
      (if turn (llif::call-take-pose-action 4))
      ?successfull-pose)
      ))

;;author Philipp Klein
(defun point-in-polygon (edges-list point)
  "return if the point is in the polygon
  `listOfEdges' the list of all edges of the polygon
  `point' the point to check if it is in the polygon"
  (setq poly-corner-1  (- (length edges-list) 1))
  (setq is-poly nil)
  (loop
    for i from 0 to (- (length edges-list) 1)
    do
       (if
        (not(and
             (> (cl-tf::y (nth i edges-list))
                (cl-tf::y point))
             (> (cl-tf::y (nth poly-corner-1 edges-list))
                (cl-tf::y point))))
        (if
            (< (cl-tf::x (nth i edges-list))
               (+ (cl-tf::x (nth i edges-list))
                  (/
                   (* (- (cl-tf::x (nth poly-corner-1 edges-list))
                         (cl-tf::x (nth i edges-list)))
                      (-(cl-tf::y point)
                        (cl-tf::y (nth i edges-list))))
                   (- (cl-tf::y (nth poly-corner-1 edges-list))
                      (cl-tf::y (nth i edges-list))))))
            (setq is-poly (not is-poly))
            (roslisp::ros-info (point-in-polygon) "Hit"))
            (setq poly-corner-1 i)))
  (roslisp::ros-info (point-in-polygon) "Next edge found: ~a" is-poly)
  is-poly)

;;source: https://stackoverflow.com/questions/2680864/how-to-remove-nested-parentheses-in-lisp
(defun flatten (l)
  "flatten a given list"
  (cond ((null l) nil)
        ((atom l) (list l))
        (t (loop for a in l appending (flatten a)))))

(defun create-move-position-list(object-id)
    (setq *pose* (llif:prolog-object-pose object-id))
  (let ((?nav-pose
          (list (cl-tf::make-pose-stamped
                 "map" 0 
                 (cl-tf:make-3d-vector
                  (- (nth 0 (nth 2 *pose*)) 0.5) ;;x-cordinate
                  (nth 1 (nth 2 *pose*))  ;;y-cordinate
                  0) 
                 (cl-tf::make-quaternion 0 0 0 1)) 
                (cl-tf::make-pose-stamped
                 "map" 0 
                 (cl-tf:make-3d-vector
                  (+ (nth 0 (nth 2 *pose*)) 0.5) 
                  (nth 1 (nth 2 *pose*))
                  3.14) 
                 (cl-tf::make-quaternion 0 0 0 1))
                (cl-tf::make-pose-stamped
                 "map" 0 
                 (cl-tf:make-3d-vector
                  (nth 0 (nth 2 *pose*))
                  (+ (nth 1 (nth 2 *pose*)) 0.5)
                  1.57)
                          (cl-tf::make-quaternion 0 0 0 1))
                (cl-tf::make-pose-stamped
                 "map" 0 
                 (cl-tf:make-3d-vector
                  (nth 0 (nth 2 *pose*))
                  (- (nth 1 (nth 2 *pose*)) 0.5)
                  -1.57)
                 (cl-tf::make-quaternion 0 0 0 1)))))
    ?nav-pose))
