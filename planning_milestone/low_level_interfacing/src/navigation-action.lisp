(in-package : llif)

;; (defvar *navp-client* nil)

;; (defun init-action-client ()
;;  (setf *navp-client* (actionlib:make-action-client
;;                       "/move_base/move"
;;                       "move_base_msgs/MoveBaseAction"))
;;  (roslisp:ros-info (move-base-action-client) 
;;                    "Waiting for Navigation action server...")
  ;; workaround for race condition in actionlib wait-for server
;;  (loop until
;;    (actionlib:wait-for-server *navp-client*))
;;  (roslisp:ros-info (move-base-action-client) 
;;                    "Navigation action client created."))

;; (defun get-action-client ()
;;  (when (null *navp-client*)
;;    (init-action-client))
;;  *navp-client*)

;; (defun make-move-action-goal (in-edges in-radius)
;; requiered:	header.frame_id = map 	
;;		pose.position:=(x,y,z)
;;		pose.orientation:=Qauternion(0, 0, 1, w) oder Euler(0, 0, w)
;;  (actionlib:make-action-goal (get-action-client)
;;                        edges in-edges
;;                        radius in-radius))

;; (defun call-shape-action (&key edges radius)
;;  (multiple-value-bind (result status)
;;      (let ((actionlib:*action-server-timeout* 10.0))
;;        (actionlib:call-goal
;;         (get-action-client)
;;         (make-shape-action-goal edges radius)))
;;    (roslisp:ros-info (turtle-shape-action-client) "Nav action finished.")
;;    (values result status)))
