(in-package :clean)

(defvar *goallist*)
(defvar *POI-list* NIL)
(defvar *next-object-list* NIL)
(defvar *next-object*)

;;@Author Jan Schimpf
(defun execute-cleanup()
  ;; Loop over goal list
    ;; Move to goal one
    ;; Scan goal
  (scan-goals)

  ;; Loop until list POI nil
    ;; Go to POI
    ;; Move to POI
    ;; Scan for objects
    ;; If objects found
    ;; get next object 
  (loop for POI in *POI-list*
        do (
            comf:move-to-poi-and-scan
            ;;(setq *next-object-list* (llif:next-object))
            
      ;; Loop until (nextobject) nil
      ;; get next object
      ;; move into position to grasp next object
      ;; grasp next object
      ;; move to the goal of the object
      ;; place object at goal
      ;; update knowledge about new position of object
      (if (not (null *next-object-list*))
      (transport-object-loop))
        ))

     
    ;; remove POI from list
  )

;;@Author Jan Schimpf
;; todo replace setq **goallist* with either a complet list of the correct goals
;; or get a list of goals from knowledge 
(defun scan-goals ()
  (setf *goallist* (list (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 2 3 0) 
                                               (cl-tf::make-quaternion 0 0 0 1))))
  (mapcar 
    (lambda (arg)
    (;;comf:hsr-move arg)
     ;;this should not be a direct call but a (move-hsr(i))
     ;;move-hsr but it at this point doesn't take a argument
     comf:scan-object)) 
     *goallist*)
  )

(defun transport-object-loop ()
   (loop while (not *next-object-list*)
         do(
            ;;(setf next-object (car (*next-object-list*)))
            comf:hsr-grasp (*next-object*)
            ;;(setf *next-object-list* (llif:next-object))
            (print "blub")
                     )))
