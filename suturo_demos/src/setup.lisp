(in-package :su-demos)

;; roslaunch sutuuro_demos ....launch
;; Add all necessary init functions for suturo here, starting with the startup function
(defun init-projection ()
  ;; (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  (setf cram-tf:*tf-default-timeout* 0.5) ; projection tf is very fast

  (coe:clear-belief)

  (setf prolog:*break-on-lisp-errors* t)

  (btr:clear-costmap-vis-object)



(roslisp-utilities:register-ros-init-function init-projection)
)
