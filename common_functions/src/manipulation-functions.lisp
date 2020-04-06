(in-package :comf)


(defparameter *goal* nil)
(defparameter *classgrasp* nil)
(defparameter *dimensions* nil)
(defparameter *pose* nil)
(defparameter *classplace* nil)
(defparameter *list* nil)
(defparameter *result* nil)
(defparameter *place-list* nil)

;;@author Jan Schimpf
;; Gets the object-id of the object that should be place and
;; the grasp pose for how the object was grasped.
;; Uses the object id to get needed information about the object
;; from knowledge and then uses this information to construct place motion-designator 

(defun place-object (object-id grasp-pose)
  ;;get the information from knowledge
  (setf *dimensions* (llif:prolog-object-dimensions object-id))
  (setf *goal* (llif:prolog-object-goal-pose object-id))
  
  ;;takes apart the messages for the needed information to consturct the place motion-designator 
  (let* ((?point-x-object (nth 0 (nth 0 *goal*)))
        (?point-y-object (nth 1 (nth 0 *goal*)))
        (?point-z-object (- (nth 2 (nth 0 *goal*)) 0.07))
        (?quaterion-value-1 (nth 0 (nth 1 *goal*)))
        (?quaterion-value-2 (nth 1 (nth 1 *goal*)))
        (?quaterion-value-3 (nth 2 (nth 1 *goal*)))
        (?quaterion-value-4 (nth 3 (nth 1 *goal*)))
        (?size-x (nth 0 *dimensions*))
        (?size-y (nth 1 *dimensions*))
        (?size-z (nth 2 *dimensions*))
        (?object-id object-id)
        (?grasp-mode grasp-pose)
   ;; construct the motion designator      
   (place (desig:a motion
                    (:type :placing)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))
    ;; executes the motion designator for place
    (print "Was here")
    (exe:perform place)))

;;@author Jan Schimpf
;; gets a list and then takes it apart and uses the information
;; to create a motion-place-designator 
(defun place-object-list (place-list)
  (setf *place-list* place-list)
  (let* ((?point-x-object (nth 0 *place-list*))
        (?point-y-object (nth 1 *place-list*))
        (?point-z-object (nth 3 *place-list*))
        (?quaterion-value-1 (nth 4 *place-list*))
        (?quaterion-value-2 (nth 5 *place-list*))
        (?quaterion-value-3 (nth 6 *place-list*))
        (?quaterion-value-4 (nth 7 *place-list*))
        (?size-x (nth 8 *place-list*))
        (?size-y (nth 9 *place-list*))
        (?size-z (nth 10 *place-list*))
        (?object-id (nth 11 *place-list*))
        (?grasp-mode(nth 12 *place-list*))
   (place (desig:a motion
                    (:type :placing)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))
    (comf::with-hsr-process-modules (exe:perform
                                     place))))  

;;@author Jan Schimpf
;;todo add checks for nil;
;;

(defun grasp-object (object-id grasp-pose)
   ;;get the information from knowledge

  (setq *dimensions* (llif:prolog-object-dimensions object-id))
  (setq *pose* (llif:prolog-object-pose object-id))
  
  ;;takes apart the messages for the needed information to consturct the grasp motion-designator 
  (let* ((?point-x-object (nth 0 (nth 2 *pose*)))
        (?point-y-object (nth 1 (nth 2 *pose*)))
        (?point-z-object (nth 2 (nth 2 *pose*)))
        (?quaterion-value-1 (nth 0 (nth 3 *pose*)))
        (?quaterion-value-2 (nth 1 (nth 3 *pose*)))
        (?quaterion-value-3 (nth 2 (nth 3 *pose*)))
        (?quaterion-value-4 (nth 3 (nth 3 *pose*)))
        (?size-x (nth 0 *dimensions*))
        (?size-y (nth 1 *dimensions*))
        (?size-z (nth 2 *dimensions*))
        (?object-id object-id)
        (?grasp-mode grasp-pose)

        ;; construct the motion designator      
        (grasp (desig:a motion
                    (:type :grasping)
                    (:point-x ?point-x-object)
                    (:point-y ?point-y-object)
                    (:point-z ?point-z-object)
                    (:quaterion-value-1 ?quaterion-value-1)
                    (:quaterion-value-2 ?quaterion-value-2)
                    (:quaterion-value-3 ?quaterion-value-3)
                    (:quaterion-value-4 ?quaterion-value-4)
                    (:size-x ?size-x)
                    (:size-y ?size-y)
                    (:size-z ?size-z)
                    (:object-id ?object-id)
                    (:grasp-mode ?grasp-mode))))

    ;; executes the motion designator for grasp
    (format t "desig: ~a" grasp)
    (exe:perform grasp)))
                                          

;;@author Jan Schimpf
;; creates a list of alternative positions for the object in case the object could not
;; be placed at the orignal position
(defun create-place-list (object-id grasp-pose)
  
  ;;get the information from knowledge
  (setf *dimensions* (llif:prolog-object-dimensions object-id))
  (setf *goal* (llif:prolog-object-goal-pose object-id))
  
  ;;takes apart the messages and then uses the information to create list with alternative options 
  (let* ((point-x-object (nth 0 (nth 0 *goal*)))
        (point-y-object (nth 1 (nth 0 *goal*)))
        (point-z-object (nth 2 (nth 0 *goal*)))
        (quaterion-value-1 (nth 0 (nth 1 *goal*)))
        (quaterion-value-2 (nth 1 (nth 1 *goal*)))
        (quaterion-value-3 (nth 2 (nth 1 *goal*)))
        (quaterion-value-4 (nth 3 (nth 1 *goal*)))
        (size-x (nth 0 *dimensions*))
        (size-y (nth 1 *dimensions*))
        (size-z (nth 2 *dimensions*))
        (?list (list (list point-x-object point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list (+ point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list (- point-x-object 0.05) point-y-object point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list point-x-object (+ point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose)
                        (list point-x-object (- point-y-object 0.05) point-z-object
                                quaterion-value-1 quaterion-value-2
                                quaterion-value-3 quaterion-value-4
                                size-x size-y size-z object-id grasp-pose))))   
    ?list))

