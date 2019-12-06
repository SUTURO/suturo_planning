(defpackage :low-level-interfaces
  (:nicknames :lli)
  (:use :roslisp :cl)
  (:export
   ;; navigation client
   #:init-nav-client
   #:get-nav-action-client
   #:make-nav-action-goal
   #:call-nav-action
   #:smash-into-appartment
   #:call-nav-action-ps
   
   ;; move client (Giskard)
   #:make-giskard-joints-action-client
   #:make-giskard-poses-action-client
   #:init-giskard-joints-action-client
   #:make-giskard-joints-action-goal
   #:call-giskard-joints-move-action
   #:call-giskard-joints-grasping-action

   ;; perception client
   #:init-robosherlock-action-client ;; only for init function in main
   #:call-robosherlock-pipeline
   #:init-robosherlock-door-action-client
   #:call-robosherlock-door-pipeline

   ;; prolog client
   #:with-safe-prolog
   #:object-name->class
   #:prolog-table-objects
   #:prolog-object-goal
   #:prolog-all-objects-in-shelf
   #:prolog-object-dimensions
   #:prolog-object-in-gripper
   #:prolog-object-goal-pose
   #:knowrob-symbol->string
   #:prolog-objects-around-pose
   #:dummy-test

   ;; joint state client
   :*start-signal-fluent*
   #:init-gripper-tilt-fluent
   #:get-current-joint-state

   ;; torso client
   #:init-move-torso-action-client
   #:call-move-torso-action
   #:test-move-torso

   ;; head client (HSR default)
   #:get-move-head-client
   :*move-head-action-timeout*
   #:init-move-head-action-client
   #:make-move-head-action-goal
   #:call-move-head-action

   ;; marker publisher
   #:init-marker-publisher
   #:get-marker-publisher
   #:publish-marker-pose

   ;; text to speech client
   :*enable-speech*
   :*text-to-speech-action-client*
   #:init-text-to-speech-action-client
   #:get-text-to-speech-action-client
   #:make-text-action-goal
   #:call-text-to-speech-action

   ;;vizbox
   #:viz-box-init
   #:publish-challenge-step
   #:publish-robot-text
   #:publish-operator-text
   ))
