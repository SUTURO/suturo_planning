(in-package :comf)
(defvar *place-list* nil)
(defparameter *listOfPoi* Nil)

;;;; Navigation ;;;;
(defun try-movement () (let 
	((?nav-pose (list (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 2 3 0) 
                                               (cl-tf::make-quaternion 0 0 0 1)) 
                     (cl-tf::make-pose-stamped "map" 0 
                                                (cl-tf:make-3d-vector 2 3 0) 
                                                (cl-tf::make-quaternion 0 0 0 1))
                     (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 1.097 0.556 0)
                                               (cl-tf::make-quaternion 0 0 0 1)))))

(urdf-proj:with-simulated-robot (cpl:with-retry-counters ((going-retry 3))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)
               (e)
             
             (setf ?nav-pose (cdr ?nav-pose))
             
             (cpl:do-retry going-retry
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%Failed to move to given position~%")
               (cpl:retry))
             
             (roslisp:ros-warn (going-demo movement-fail)
                               "~%No more retries~%")))
        
          (let ((?actual-nav-pose (car ?nav-pose))) 
          (cram-executive:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?actual-nav-pose)))))
            ?actual-nav-pose))))))


;;;;;;;;;;;;;;;;;;;;Try Movement with List ;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun try-movement-stampedList (listStamped)
  (let (?nav-pose listStamped)

    (urdf-proj:with-simulated-robot
      (cpl:with-retry-counters ((going-retry 3))
        (cpl:with-failure-handling
            (((or common-fail:low-level-failure 
                  cl::simple-error
                  cl::simple-type-error)
                 (e)
               
               (setf ?nav-pose (cdr ?nav-pose))
             
               (cpl:do-retry going-retry
                 (roslisp:ros-warn (going-demo movement-fail)
                                   "~%Failed to move to given position~%")
                 (cpl:retry))
             
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%No more retries~%")))
        
          (let ((?actual-nav-pose (car ?nav-pose))) 
          (cram-executive:perform
           (desig:an action
                     (type going)
                     (target (desig:a location (pose ?actual-nav-pose)))))
            ?actual-nav-pose))))))
          
(defun move-hsr ()
  (let* ((?successfull-pose (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector 1.097 0.556 0)
                                               (cl-tf::make-quaternion 0 0 0 1)))
        (?desig (desig:a motion
                        (type going) 
                        (target (desig:a location
                                         (pose ?successfull-pose))))))
    
    (llif::with-hsr-process-modules
      (exe:perform ?desig))
))

;;;; Grasp ;;;;;
;; (defun try-grasp (position) 
;;   (let ((?grasp position)
       
;;         (urdf-proj:with-simulated-robot
;;           (cpl:with-retry-counters ((grasping-retry 3))
;;             (cpl:with-failure-handling
;;                 (((or common-fail:low-level-failure 
;;                       cl::simple-error
;;                       cl::simple-type-error)                
;;                      (e)
;;                    (setf ?grasp (cdr ?grasp))
;;                    (cpl:do-retry grasping-retry
;;                      (roslisp:ros-warn (going-demo movement-fail)
;;                                        "~%Failed to grasp object~%")
;;                      (cpl:retry))
;;                    (roslisp:ros-warn (going-demo movement-fail)
;;                                      "~%No more retries~%")))
;;               (let ((?current-grasp-position (car ?grasp)))
                
;;                 (cram-executive:perform
;;                  (desig:an action
;;                            (type grasping)
;;                            (target (desig:a location (pose ?current-grasp-position)))))
;;                 ?current-grasp-position)))))))


(defun grasp-hsr (object-id grasp-pose)    
  (cpl:with-retry-counters ((grasping-retry 3))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
         (scan-object
          ;;closet function i currently know of that allows me
          ;;to trigger perception for this
          )
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))

         (roslisp:ros-warn (going-demo movement-fail)
                            "~%No more retries~%")))
 (comf:grasp-object object-id grasp-pose))))

;;;; Place ;;;;
(defun place-hsr (object-id grasp-pose)
  (let ((?place-position (comf:create-place-list object-id grasp-pose)))

                         
 (cpl:with-retry-counters ((grasping-retry 4))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (setf ?place-position (cdr ?place-position))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
         (let ((?actual-place-position (car ?place-position)))
           (llif::with-hsr-process-modules
             (exe:perform (desig:a motion (type placeing)
                                   (target (desig:a location
                                                    (pose ?actual-place-position))))))
                       )))))))
   

;;;;Move-to-Grasp;;;;
(defun move-to-grasp(object-id grasp-mode)
  (let ((?move-positions (create-move-position-list object-id)))
   (cpl:with-retry-counters ((grasping-retry 4))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (setf ?move-positions (cdr ?move-positions))
       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))
    ;; (let ((?successfull-pose (try-movement (car move-positions))))
    ;; (try-grasp ((llif:prolog-object-pose object-id)))
    ;; (llif::with-hsr-process-modules (exe:perform (desig:a motion (type going) 
    ;; (target (desig:a location (pose ?successfull-pose))))))
    ;; (llif::with-hsr-process-modules (exe:perform (desig:a motion (type grasping)
    ;; (target (desig:a location (grasp-object object-id grasp-mode)))))
         ))))))


(defun move-to-poi ()
        ;;Point to go is: goal + (poiDistance/distance)*(currentpose - goal)
	;;please indent region...
	
        (setf *listOfPoi* (mapcar (lambda (listelem) (comf::pose-with-distance-to-point *poiDistance* listelem)) 
                                 (llif::sortedPoiByDistance
					(cl-tf::transform-stamped->pose-stamped
					   (cl-tf::lookup-transform  cram-tf::*transformer*  "map" "base_footprint")))))
        (try-movement-stampedList *listOfPoi*)
        (llif::call-take-pose-action 2))
