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

   ;; perception-trigger
   #:trigger-perception-pipeline-main
   
   ;; perceive-action-client
   #:init-perceive-action-client
   #:get-perceive-action-client
   #:make-perceive-action-goal
   #:ensure-perceive-goal-reached

   ;;grasp-action-client
   #:init-grasp-action-client
   #:get-grasp-action-client
   #:make-grasp-action-goal
   #:ensure-grasp-goal-reached
   #:call-grasp-action
   #:test-grasp-action 
   ))
