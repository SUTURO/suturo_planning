(in-package :comp)

;;call-move-gripper-action
;;make-perceive-action-goal
;;call-grasp-action
;;call-place-action
;;
;;

(defun move-to-perception-point(px py pe)
  (llif::call-nav-action px py pe)
)

(defun find-object()
(llif::trigger-perception-pipeline-main)
)

(defun grasp-object(px py pz ox oy oz ow size_x size_y size_z)
(llif:call-grasp-action px py pz ox oy oz ow size_x size_y size_z)
)

(defun move-to-goal(px py pe)
 (llif::call-nav-action px py pe)
)

(defun place-object(px py pz ox oy oz ow)
 (llif:call-place-action px py pz ox oy oz ow)
)

(defun execute-m1()
  (init-manipulation)
       (llif:listener)
       (find-object)
;;     (grasp-object px py pz ox oy oz ow size_x size_y size_z)
       (place-object 1.0 0.6865 0.28 0.0 0.7 0.0 0.7)
)
