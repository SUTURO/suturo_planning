(in-package :comp)

(defun make-poi (px py pz ox oy oz ow)
	(cl-transforms-stamped:to-msg
                (cl-tf::make-pose-stamped  
                 "map"
                 0.0 
                 (cl-tf:make-3d-vector px py pz) 
                 (cl-tf:make-quaternion ox oy oz ow))))

(defparameter *poi* (list
	(make-poi 0 0 0 0 0.7 0 0.7)
        (make-poi 0 0 0 0 0.7 0 0.7)))

(defun add-poi (px py pz ox oy oz ow)
	(defparameter *poi* (append *poi* (list(make-poi px py pz ox oy oz ow))))
        ;;(append *poi* (make-poi px py pz ox oy oz ow))
)


(cpl:def-cram-function move-to-poi ()
	(llif::call-nav-action-ps (pop *poi*))
)

;; Noch den confidence Filter hinzufügen!!
(cpl:def-cram-function scan-object ()
	(llif::insert-knowledge-objects(llif::call-robosherlock-pipeline))
	)

(cpl:def-cram-function move-to-poi-and-scan ()
	(move-to-poi)
	(scan-object)
)
