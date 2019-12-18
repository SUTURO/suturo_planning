(in-package :comp)

(defun move-to-table()
  (llif::call-nav-action 0 0 0)
)

(defun find-object()
(call-robosherlock-pipeline)
)

(defun grasp-object()

)

(defun move-to-goal()
 (llif::call-nav-action 0 0 0)
)

(defun place-object()

)

(defun execute-m1()
       (move-totable)
       (find-object)
       (grasp-object)
       (move-to-goal)
       (place-object)
)
