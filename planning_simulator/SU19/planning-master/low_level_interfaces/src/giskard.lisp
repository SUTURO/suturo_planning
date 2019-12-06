(in-package :lli)
;; probably deprecated

;; (defvar *giskard-action-client* nil)

;; (defparameter *giskard-action-timeout* 20.0 "in seconds")

;; (defun init-giskard-action-client ()
;;   (setf *giskard-action-client* (actionlib:make-action-client
;;                                  "do_move_poses"
;;                                  "move/DoMovePosesAction"))
;;   (loop until
;;         (actionlib:wait-for-server *giskard-action-client*))
;;   (roslisp:ros-info (giskard-action-client)
;;                     "Giskard action client created."))

;; (defun destroy-giskard-action-client ()
;;   (setf *giskard-action-client* nil))

;; (roslisp-utilities:register-ros-cleanup-function destroy-giskard-action-client)

;; (defun get-action-client ()
;;   (when (null *giskard-action-client*)
;;     (init-giskard-action-client))
;;   *giskard-action-client*)

;; ;; TODO: define goal
;; ;(defun make-giskard-do-move-action-goal (text)
;; ;  (actionlib:make-action-goal (get-action-client)
;; ;    :data (make-message "move/DoMoveJointsAction"
;; ;                        :goal_msg text
;; ;                        :desired_joints_values nil)))

;; (defun make-test-pose ()
;;   (cl-tf:make-pose (cl-tf:make-3d-vector 0 0 0)
;;                    (cl-tf:make-identity-rotation)))

;; (defun make-test-pose-stamped (x y z)
;;   (cl-tf:make-pose-stamped "wrist_roll_link"
;;                                (roslisp:ros-time)
;;                                (cl-tf:make-3d-vector x y z)
;;                                (cl-tf:make-identity-rotation)))

;; (defun make-test-poses (x y z)
;;   (list (make-test-pose-stamped x y z)))

;; (defun make-do-move-poses-action-goal (text list-poses object-pose)
;;   (actionlib:make-action-goal (get-action-client)
;;     goal_msg text
;;     list_poses (cl-tf:to-msg list-poses)
;;     object_pose (cl-tf:to-msg object-pose)))

;; (defun call-giskard-do-move-action (text list-poses object-pose)
;;   (multiple-value-bind (result status)
;;       (let ((actionlib:*action-server-timeout* 10.0))
;;         (actionlib:call-goal
;;          (get-action-client)
;;          (make-do-move-poses-action-goal text list-poses object-pose)))
;;     (roslisp:ros-info (giskard-action-client) "do_move_poses action finished.")
;;     (values result status)))


;; (defun test-single-pose (x y z)
;;   ;; TODO: in test-file verschieben
;;   (call-giskard-do-move-action "Test single pose"
;;                                (make-test-poses x y z)
;;                                (make-test-pose)))

;; TODO: - test file mit funktionen wie test-single-pose
;;       - diesen file refactoren (benennung anpassen, parameter ev anpassen/benennen)
;;       - einen action client für do_move_joints implementieren
;;       Ziel: in goal_msg: "grip" oder "move"
;;             - wenn "grip" gegeben: list_poses leer, object_pose gegeben
;;             - wenn "move" gegeben: list_poses gegeben, object_pose leer
;;
;; object_pose sollte in relation zum roboter, nicht zur world sein
