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

   ;; Perception-trigger
   #:trigger-perception-pipeline-main
   ))
