(in-package :lli)
;; probably deprecated
;; (defparameter *giskard-poses-action-timeout* 300.0 "in seconds")

;; (defun make-giskard-poses-action-client ()
;;   (cram-simple-actionlib-client:make-simple-action-client
;;    'move-poses-action
;;    "do_move_poses" "move/DoMovePosesAction"
;;    *giskard-poses-action-timeout*
;;    :initialize-now T))

;; ;; (roslisp-utilities:register-ros-init-function make-giskard-poses-action-client)

;; (defun make-giskard-poses-action-goal (text &key
;;                                               (poses NIL)
;;                                               (object-pose (cl-tf:make-identity-pose)))
;;   (actionlib:make-action-goal
;;       (cram-simple-actionlib-client::get-simple-action-client 'move-poses-action)
;;     goal_msg text
;;     list_poses (cl-tf:to-msg poses)
;;     object_pose (cl-tf:to-msg object-pose)))

;; (defun ensure-giskard-poses-grasping-input (object-pose)
;;   ;; TODO: check if object-pose is possible to grasp, e.g. check if it is to wide
;;   object-pose
;;   T)

;; (defun ensure-giskard-poses-move-input (poses)
;;   ;; TODO: check if poses are possible to reach, e.g. check if they are to high...
;;   poses
;;   T)

;; (defun ensure-giskard-poses-grasping-goal-reached (status object-pose)
;;   ;; TODO: check status if given object-pose is reached
;;   status
;;   object-pose  
;;   T
;; )

;; (defun ensure-giskard-poses-move-goal-reached (status poses)
;;   ;; TODO: check status if given poses are reached
;;   (roslisp:ros-warn (move-poses-action) "Status: ~a" status)
;;   status
;;   poses  
;;   T
;; )

;; (defun call-giskard-poses-grasping-action (object-pose)
;;   (when (ensure-giskard-poses-grasping-input object-pose)
;;     (multiple-value-bind (result status)
;;       (cram-simple-actionlib-client::call-simple-action-client
;;        'move-poses-action
;;        :action-goal (make-giskard-poses-action-goal "grasp" :object-pose object-pose)
;;        :action-timeout *giskard-poses-action-timeout*)
;;       (roslisp:ros-info (move-poses-action) "do_move_poses grasp action finished.")
;;       (ensure-giskard-poses-grasping-goal-reached status object-pose)
;;       (values result status))))

;; (defun call-giskard-poses-move-action (poses)
;;   (when (ensure-giskard-poses-move-input poses)
;;     (multiple-value-bind (result status)
;;       (cram-simple-actionlib-client::call-simple-action-client
;;        'move-poses-action
;;        :action-goal (make-giskard-poses-action-goal "move" :poses poses)
;;        :action-timeout *giskard-poses-action-timeout*)
;;       (roslisp:ros-info (move-poses-action) "do_move_poses move action finished.")
;;       (ensure-giskard-poses-move-goal-reached status poses)
;;       (values result status))))

