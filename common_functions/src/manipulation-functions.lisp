(in-package :comf)

;; used in cleanup
;;@author Jan Schimpf
;; Gets the object-id of the object that should be place and
;; the grasp pose for how the object was grasped.
;; Uses the object id to get needed information about the object
;; from knowledge and then uses this information to construct place motion-designator 
(defun place-object (object-id grasp-pose &optional (mode :cleanup))
  ;;get the information from knowledge
  
  (let* ((object-dimensions (llif:prolog-object-dimensions object-id))
         (prolog-goal-pose (if (equalp mode :cleanup)
                               (llif:prolog-object-goal-pose object-id)
                               (llif:prolog-temporary-storage-pose object-id)))
         (object-pose (list (first prolog-goal-pose)
                            (second prolog-goal-pose))))
     (print "2")
    (roslisp::ros-info (place-object) "Placing at ~a" (car object-pose))
    ;;takes apart the messages for the needed information to consturct the place motion-designator 
    (let* ((?point-x-object (first (first object-pose)))
           (?point-y-object (second (first object-pose)))
           (?point-z-object (third (first object-pose)))
           (?quaterion-value-1 (first (second object-pose)))
           (?quaterion-value-2 (second (second object-pose)))
           (?quaterion-value-3 (third (second object-pose)))
           (?quaterion-value-4 (fourth (second object-pose)))
           (?size-x (first object-dimensions))
           (?size-y (second object-dimensions))       
           (?size-z (third object-dimensions))
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
      (ros-info (place-object) "Placing with ~a" place)
      (print place)
      
      ;; executes the motion designator for place
      (exe:perform place))))






;; used in cleanup
;;@author Jan Schimpf
;; Gets the object-id of the object that should be grasped and
;; the grasp pose for how the object should be grasped
;; Uses the object id to get needed information about the object
(defun grasp-object (object-id grasp-pose)
  ;;get the information from knowledge
  (let* ((object-dimensions (llif:prolog-object-dimensions object-id))
         (raw-object-pose (llif:prolog-object-pose object-id))
         (object-pose (list (third raw-object-pose)
                            (fourth raw-object-pose))))
    (roslisp::ros-info (grasp-object) "Grasping at ~a" (car object-pose))
    ;;takes apart the messages for the needed information to consturct the grasp motion-designator 
    (let* ((?point-x-object (first (first object-pose)))
           (?point-y-object (second (first object-pose)))
           (?point-z-object (third (first object-pose)))
           (?quaterion-value-1 (first (second object-pose)))
           (?quaterion-value-2 (second (second object-pose)))
           (?quaterion-value-3 (third (second object-pose)))
           (?quaterion-value-4 (fourth (second object-pose)))
           (?size-x (first object-dimensions))
           (?size-y (second object-dimensions))
           (?size-z (third object-dimensions))
           (?object-id object-id)
           (?grasp-mode grasp-pose)
        
           ;; construct the motion designator      
           (grasp
             (desig:a motion
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
      (exe:perform grasp))))
                                          
