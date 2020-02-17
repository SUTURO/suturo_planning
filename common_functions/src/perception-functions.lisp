(in-package :comf)

(defvar *objects* NIL)
;;(defvar *x* NIL)
(defvar *object-list* NIL)
(defvar *pipeline-msg* NIL)

;;(defun get-confident-objects ()
;;  (setf *objects* (llif:call-robosherlock-pipeline))
;;  (let* (?confidence (roslisp:slot-value objects :confidence))
;;    (?objects (delete-if (lambda (arg) (confidence)) objects))))

(defun get-confident-objects ()
  (setq *objects* (roslisp::ros-message-to-list (llif::call-robosherlock-pipeline))) 
  (roslisp::list-to-ros-message (roslisp::with-fields (detectiondata) *objects* (delete-if (lambda (obj)
   (roslisp::with-fields (confidence_class) obj (> confidence_class 0.4)) *objects*) *objects*))))

;;(defun get-confident-objects (&optional regions-value)
;;  (setf *pipeline-msg* (llif:call-robosherlock-pipeline regions-value))
;;  (roslisp::with-fields (detectiondata) *pipeline-msg* (setf *object-list* detectiondata))
;;  (roslisp::list-to-ros-message (delete-if (lambda (object)
;;  (roslisp::with-fields (confidence_class) object (> confidence_class 0.55)) *object-list*)
;;  *object-list*))))										      
