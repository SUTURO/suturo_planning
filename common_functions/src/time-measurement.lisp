(in-package :comf)

(defparameter *total-time* 0)
(defparameter *total-init-time* 0)
(defparameter *total-movement-time* 0)
(defparameter *total-manipulation-time* 0)
(defparameter *total-detection-time* 0)

;; used in cleanup
;; @author Luca Krohm
(defmacro measure-time (type &body body)
  "Receives a type `type', which is either :start, :init, :movement, :manipulation or :detection and a body of lisp code `body'. Macro to measure the time an action takes to finish, returns the returnvalue of the body"
  `(let ((start (get-universal-time))
         (value (progn ,@body)))
     ,(ccase type
        (:start  '(calculate-time start '*total-time*))
        (:init '(calculate-time start '*total-init-time*))
        (:movement  '(calculate-time start '*total-movement-time*))
        (:manipulation  '(calculate-time start '*total-manipulation-time*))
        (:detection  '(calculate-time start '*total-detection-time*)))
     value))

;; used in cleanup
;;@author Luca Krohm
(defun calculate-time (timestamp total)
  "Receives timestamp `timestamp' and global variable `total'. Calculates time passed since `timestamp' and adds it to `total'."
  (roslisp::ros-info (time-measurement)
                     "measuring time ~a"
                     total)
  (set total
        (+
         (- (get-universal-time) timestamp)
         (eval total))))

;; used in cleanup
;; author Luca Krohm
(defun reset-timestamps ()
  "Resets all global timestamp-variables."
  (setf *total-time* 0)
  (setf *total-init-time* 0)
  (setf *total-movement-time* 0)
  (setf *total-manipulation-time* 0)
  (setf *total-detection-time* 0))

;; used in cleanup
;;@author Luca Krohm
(defun announce-final-time ()
  "Prints out a message containing the global variables used to measure the time."
  (roslisp::ros-info (execute-cleanup)
                     "Plan execution took ~a seconds"
                     *total-time*)
  (roslisp::ros-info (execute-cleanup)
                     "Of that ~a seconds were used for initialisation, ~a seconds were used for detection ~a seconds for movement and ~a seconds for manipulation actions the remaining ~a seconds were used for querying for information from the database"
                     *total-init-time*
                     *total-detection-time*
                     *total-movement-time*
                     *total-manipulation-time*
                     (- *total-time*
                        *total-detection-time*
                        *total-movement-time*
                        *total-manipulation-time*
                        *total-init-time*)))
