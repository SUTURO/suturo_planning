(in-package :clean)

(defvar *goallist*)
(defvar *object-id*)
(defvar *table-objects* NIL)

(defun execute-cleanup()
  ;; Loop over goal list
  ;; Move to goal one
  ;; Scan goal
  (setq *goallist* (list (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 2 3 0) 
                                               (cl-tf::make-quaternion 0 0 0 1))))
  (mapcar
   (lambda (arg)
    (llif::call-nav-action-ps arg)
     ;;this should not be a direct call but a (move-hsr(i))
     ;;move-hsr but it at this point doesn't take a argument
     (comf:scan-object)) 
     *goallist*)
  
  ;; Loop until list POI nil
    ;; Go to POI
    ;; Move to POI
    ;; Scan for objects
    ;; If objects found
    ;; get next object
      ;; Loop until (nextobject) nil
      ;; get next object
      ;; move into position to grasp next object
      ;; grasp next object
      ;; move to the goal of the object
      ;; place object at goal
      ;; update knowledge about new position of object
    ;; remove POI from list
  )
