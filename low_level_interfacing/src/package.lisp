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

   ;;perceive-action-client 
   #:init-perceive-action-client
   #:get-perceive-action-client 
   #:call-perceive-action 
   #:make-perceive-action-goal
   #:ensure-perceive-goal-reached 

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
   #:object-name->class 
))
