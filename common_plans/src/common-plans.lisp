(in-package :comp)

(defvar *objects* NIL)
(defvar *x* NIL)

;;(defun get-confident-objects ()
;;  (setf *objects* (llif:call-robosherlock-pipeline))
;;  (let* (?confidence (roslisp:slot-value objects :confidence))
;;    (?objects (delete-if (lambda (arg) (confidence)) objects))))

(defun get-confident-objects ()
  (setq *objects* (roslisp::ros-message-to-list (llif::call-robosherlock-pipeline))) 
  (roslisp::list-to-ros-message (delete-if (lambda (obj)
   (roslisp::with-fields (confidence) obj (> confidence 0.4)) *objects*) *objects*)))
