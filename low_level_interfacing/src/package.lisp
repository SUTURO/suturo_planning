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

   ;;make-plan-action-client
   #:init-make-plan-action-client
   #:try-make-plan-action
   #:make-plan-action-goal
   #:get-make-plan-action-client

   ;;move-gripper-client
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
   #:knowrob-symbol->string
   #:object-name->class
   #:prolog-add-test-objects
   #:prolog-all-objects-in-shelf
   #:prolog-object-in-gripper
   #:prolog-forget-table-objects
   
   ;;knowledge-door-client
   #:prolog-get-all-door-states
   #:prolog-update-doorr-state
   #:prolog-get-angle-to-open-door
   
   ;;knowledge-object-client
   #:prolog-table-objects
   #:prolog-object-goal
   #:prolog-object-supporting-surface
   #:prolog-object-goal-pose
   #:prolog-temporary-storage-pose
   #:prolog-next-object
   #:prolog-object-dimensions
   #:prolog-object-pose
   #:prolog-next-graspable-object
   #:prolog-non-graspable-objects-on-surface
   #:set-object-not-graspable
   #:get-reason-for-object-goal-pose
   #:prolog-object-room
   #:prolog-room-objects
   
   ;;knowledge-surface-client
   #:prolog-surface-pose
   #:prolog-tables
   #:prolog-shelfs
   #:prolog-buckets
   #:prolog-get-surface-regions
   #:prolog-get-surface-room
   #:prolog-current-room
   #:prolog-get-room-to-room-obstacles
   #:current-pose-prolog-pose->distance
   #:sort-surfaces-by-distance
   #:prolog-surface-region
   #:prolog-surface-room
   #:prolog-room-surfaces
   #:prolog-current-room
   #:prolog-set-surfaces-visit-state
   #:prolog-surfaces-not-visited
   #:prolog-surfaces-not-visited-in-room
   #:prolog-surface-from-urdf
   
   ;;knowledge-tts-client
   #:prolog-perceived-object->object-id
   #:prolog-perceived-room->room-id

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
