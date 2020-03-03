(in-package :comf)

;;@author Torge Olliges
;; (defun get-confident-objects (&optional (regions-value (string "robocup_table")))
;;   (setf *objects* (llif::call-robosherlock-pipeline regions-value))
;;    (roslisp-msg-protocol:list-to-ros-message
;;    (delete-if 
;;     (lambda (obj)
;;      (roslisp::with-fields 
;;       (confidence_class) obj 
;;       (if (> confidence_class 0.6) 
;;           (roslisp::ros-info 
;;             (perception-functions) "Object confidence high enough")
;;           (roslisp::ros-info 
;;             (perception-functions) "Object confidence to low"))))
;;       *objects*))
;;   *objects*)

;;@author Torge Olliges
(defun get-confident-objects(perception-msg)
 (roslisp::with-fields (detectiondata) perception-msg
  (delete-if 
   (lambda (object) 
    (roslisp::with-fields (confidence_class confidence_shape confidence_color) object 
     (if (> confidence_class 0.5) 
         (detectiondata)
         (if (> confidence_shape 0.5) 
           (detectiondata)
           (if (> confidence_color 0.5) 
               (detectiondata)
               (print "neither class shape or color confidence is high enough"))))
      )
    )
   detectiondata
  )
 )
)

;;@author Jan Schimpf
;; uses the Simulation to look at a object-position
(defun looking (object-position)
  (defparameter ?look object-position)
  (urdf-proj:with-simulated-robot (cpl:with-retry-counters ((looking-retry 3))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure
               (e)
             (declare (ignore e))
             (cpl:do-retry looking-retry
               (roslisp:ros-warn (slice-demo looking-fail)
                                 "~%Failed to look at given position~%")
               (cpl:retry))
             (roslisp:ros-warn (slice-demo looking-fail)
                               "~%No more retries~%")))
        (dotimes (n 3)
          (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?look)))))))))

  )
;;@author Jan Schimpf
;;uses the simulation to detect a object in its view 
(defun detecting ()
  (cpl:with-retry-counters ((detecting-retry 5))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (declare (ignore e))
             (cpl:do-retry detecting-retry
               (roslisp:ros-warn (slice-demo detecting-fail)
                                 "~%Failed to detect ~a~%" ?knife-type)
               (cpl:retry))
             (roslisp:ros-warn (slice-demo detecting file)
                               "~%No more retries~%")))
        (defvar ?object-mug
          (urdf-proj::detect (desig:an object (type :mug)))))) 
  ;; ToDo look into if detect can use something else then type (object-id would be best)
  )
