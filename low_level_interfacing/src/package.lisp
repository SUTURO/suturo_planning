(defpackage :low-level-interfacing
  (:nicknames :llif)
  (:use :roslisp :cl)
  (:export
   ;; navigation client
   #:init-nav-client
   #:get-nav-action-client
   #:make-nav-action-goal
   #:call-nav-action
   #:smash-into-appartment
   #:call-nav-action-ps

   ;;make-nav-plan-action-client
   #:init-make-nav-plan-action-client
   #:try-make-nav-plan-action
   #:make-nav-plan-action-goal
   #:get-make-nav-plan-action-client

   ;; move-gripper-client
   #:init-move-gripper-action-client
   #:get-move-gripper-client
   #:make-move-gripper-action-goal
   #:call-move-gripper-action
   #:ensure-move-gripper-goal-reached

   ;;grasp-action-client
   #:init-grasp-action-client
   #:get-grasp-action-client
   #:make-grasp-action-goal
   #:ensure-grasp-goal-reached
   #:call-grasp-action
   #:test-grasp-action 

   ;;place-action-client
   #:init-place-action-client
   #:get-place-action-client
   #:make-place-action-goal
   #:ensure-grasp-goal-reached
   #:call-place-action
   #:test-place-action 

   ;;take-pose-action-client 
   #:init-take-pose-action-client
   #:get-take-pose-action-client 
   #:call-take-pose-action 
   #:make-take-pose-action-goal
   #:ensure-take-pose-goal-reached 

   ;; text to speech client
   :*enable-speech*
   :*text-to-speech-action-client*
   #:init-text-to-speech-action-client
   #:get-text-to-speech-action-client
   #:make-text-action-goal
   #:call-text-to-speech-action

   ;;robosherlock-client-object.lisp
   #:init-robosherlock-object-action-client
   #:call-robosherlock-pipeline

   ;;robosherlock-client-plane.lisp
   #:init-robosherlock-plane-action-client
   #:call-robosherlock-pipeline

   ;;knowledge-insertion-client.lisp
   #:init-knowledge-action-client
   #:get-knowledge-client
   #:insert-knowledge-objects
   
   ;;knowledge-client
   #:prolog-object-pose
   #:prolog-object-in-gripper
   #:prolog-object-dimensions
   #:prolog-all-objects-in-shelf
   #:prolog-object-goal-pose
   #:prolog-object-goal
   #:prolog-table-objects 
   #:prolog-objects-around-pose
   #:object-name->class
   #:prolog-table-pose
   #:prolog-shelf-pose
   #:prolog-target-pose
   #:prolog-next-object

   ;;nlp-subscriber
   #:static-command-listener
   #:dynmaic-command-listener

   ;;poi-client
   #:sortedStampedByDistance

   ;;open-door-client
   #:get-open-door-client
   #:init-open-door-client
   #:make-open-door-goal
   #:ensure-open-goal-reached
   #:call-grasp-action
   
   ;;robosherlock-client-door
   #:init-robosherlock-door-action-client
   #:get-robosherlock-door-client
   #:make-action-goal-door
   #:call-robosherlock-door-pipeline

  ;;process-module


))
