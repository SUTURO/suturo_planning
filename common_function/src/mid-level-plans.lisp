(in-package :comp)

(defvar *objects* NIL)
(defvar *robot-pose-stamped* NIL)
(defvar *robot-transform* NIL)

(defvar *objects-list* NIL)
(defvar *confident-objects*)
(defvar *pipeline-msg* NIL)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Next Object Stuff;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun next-object (from)
    "get the next Object to grasp"
    (setq *objects* (list nil))
     (case (intern (string-upcase from))
        (table (setq *objects* (llif:prolog-table-objects)))
        (shelf (setq *objects* (llif:prolog-all-objects-in-shelf)))
        ;;(pose (setq *objects* (llif:prolog-objects-around-pose (get-pose) threshold)))
        ;;TODO: use new prolog query to knowledge)
    (if (null *objects*) 
        (error 'no-object)))
    (first *objects*))
       

(defun get-pose ()
  ;;get robot position
  (setf *robot-transform* 
        (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint"))
  ;;create robot pose
  (setf *robot-pose-stamped* (cl-transforms:make-pose 
                (cl-tf:origin *robot-pose-stamped*) 
                (cl-tf:orientation *robot-pose-stamped*))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Confident Object Stuff;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;(defun get-confident-objects ()
;;  (setf *objects* (llif:call-robosherlock-pipeline))
;;  (let* (?confidence (roslisp:slot-value objects :confidence))
;;    (?objects (delete-if (lambda (arg) (confidence)) objects))))

;;TODO: Still not done :( my fault tho
(defun get-confident-objects ()
  (setq *objects* (roslisp::ros-message-to-list (llif::call-robosherlock-pipeline))) 
  (roslisp::list-to-ros-message (roslisp::with-fields (detectiondata) *objects* (delete-if (lambda (obj)
   (roslisp::with-fields (confidence_class) obj (> confidence_class 0.4)) detectiondata) *objects*))))

;;(defun get-confident-objects (&optional regions-value)
;;  (setf *pipeline-msg* (llif:call-robosherlock-pipeline regions-value))
;;  (roslisp::with-fields (detectiondata) *pipeline-msg* (setf *object-list* detectiondata))
;;  (roslisp::list-to-ros-message (delete-if (lambda (object)
;;  (roslisp::with-fields (confidence_class) object (> confidence_class 0.55)) *object-list*)
;;  *object-list*))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Point of Interest stuff;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;(defun make-poi (px py pz ox oy oz ow)
	
(defun make-poi (px py pz ox oy oz ow)
		(cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow))))

(defparameter *poi* (list
	
        ()))

(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi* (list(make-poi px py pz ox oy oz ow))))
        ;;(append *poi* (make-poi px py pz ox oy oz ow))
)


(cpl:def-cram-function move-to-poi ()
	(llif::call-nav-action-ps (pop *poi*))
)

(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(get-confident-objects))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)
