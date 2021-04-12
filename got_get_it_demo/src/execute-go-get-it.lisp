(in-package :get-it)

(defparameter *perception-objects* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *object-id* NIL)
(defparameter *next-object* NIL)
(defparameter *stamp-pose* NIL)

(defparameter *no-objects* (cram-language:make-fluent :name :objects) NIL)
(defparameter *grasp-object-result* NIL)
(defparameter *place-object-result* NIL)
(defparameter *grasping-retries* 0)
(defparameter *graspmode* NIL)

(defun execute-go-get-it()
    (comf::with-hsr-process-modules
    ;; move to predefined location
    (move-to-start-position)
        ))

(defun move-to-start-position()
    )

(defun wait-for-orders()
    )

