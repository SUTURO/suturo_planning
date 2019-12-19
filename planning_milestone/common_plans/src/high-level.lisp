(in-package :comp)

;;call-move-gripper-action
;;make-perceive-action-goal
;;call-grasp-action
;;call-place-action
;;
;;

(defun move-to-perception-point()
  (llif::call-nav-action 0 0 0)
)

(defun find-object()
(llif:call-robosherlock-pipeline)
)

(defun grasp-object(px py pz ox oy oz ow size_x size_y size_z)
(llif:call-grasp-action px py pz ox oy oz ow size_x size_y size_z)
)

(defun move-to-goal()
 (llif::call-nav-action 0 0 0)
)

(defun place-object()
 (llif:call-place-action 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
)

(defun execute-m1()
       (llif:listener)
       (move-to-perception-point)
       (find-object)
       (grasp-object px py pz ox oy oz ow size_x size_y size_z)
       (move-to-goal)
       (place-object)
)
