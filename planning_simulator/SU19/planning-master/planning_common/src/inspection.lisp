(in-package :plc)

;;launch ros node first with:
;; (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)

;;ENTRANCE
(defparameter *poses-list* (list
                            (make-pose-stamped -1.63 0.005 1.5)
                            (make-pose-stamped -1.19 0.98 1.5)
                            (make-pose-stamped -0.97 1.99 1.5)
                            (make-pose-stamped -0.98 2.99 1.5)
                            (make-pose-stamped -0.58 3.96 1.5)
                            (make-pose-stamped -0.29 4.96 0.8)
                            (make-pose-stamped -0.18 6.13 0.0)))

;;EXIT
(defparameter *exit-list* (list
                           (make-pose-stamped 1.0 5.85 0.0)
                           (make-pose-stamped 2.065 5.497 0.0)
                           (make-pose-stamped 3.09 5.72 1.5)
                           (make-pose-stamped 3.60 6.38 0.0)
                           (make-pose-stamped 5.05 6.305 0.0)
                           ))
(defparameter *list* (list
                      (make-pose-stamped 8.16 -0.83 0.0)))


(defun inspection ()
  (lli:init-nav-client)
  (dolist (pose-stamped (subseq *poses-list* 0 5))
    (lli:call-nav-action-ps pose-stamped))
  (sleep 180)
  (dolist (pose-stamped (subseq *poses-list* 5))
    (lli::call-nav-action-ps pose-stamped)))

(defun viz-inspection ()
  (lli:get-marker-publisher)
  (mapcar (lambda (pose-stamped)
            (lli:publish-marker-pose pose-stamped :g 1.0)
            (sleep 2.0))
          *poses-list*))


(defun make-pose-stamped (x y zeuler)
  (cl-tf:make-pose-stamped
   "map"
   (roslisp::ros-time)
   (cl-tf:make-3d-vector x y 0.0)
   (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az zeuler)))

