(in-package :comf)
(defvar *place-list* nil)

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
          
(defun move-hsr ()
(let ((?successfull-pose (try-movement)))
  (llif::with-hsr-process-modules (exe:perform (desig:a motion (type going) 
  (target (desig:a location (pose ?successfull-pose))))))
))

;;;; Grasp ;;;;
;;(defun try-grasp () 
;;(let 
;;    ((?grasp (list (1 1 0.5 0 0.7 0 0.7 0.1 0.1 0.1 "box" 1)
;;                                       (1 1 0.5 0 0.7 0 0.7 0.1 0.1 0.1 "box" 1)
;;                                       (1 1 0.5 0 0.7 0 0.7 0.1 0.1 0.1 "box" 1))))
;;)))
;;(urdf-proj:with-simulated-robot (cpl:with-retry-counters ((grasping-retry 3))
;;      (cpl:with-failure-handling
;;          (((or common-fail:low-level-failure 
;;                cl::simple-error
;;                cl::simple-type-error)                
;;               (e)
;;             (setf ?grasp (cdr ?grasp))
;;             (cpl:do-retry grasping-retry
;;               (roslisp:ros-warn (going-demo movement-fail)
;;                                "~%Failed to grasp object~%")
;;              (cpl:retry))
;;            (roslisp:ros-warn (going-demo movement-fail)
;;                               "~%No more retries~%")))
;;        (let ((?actual-object-pose (car ?grasp)))
;;          
;;          (cram-executive:perform
;;           (desig:an action
;;                     (type grasping)
;;                     (target (desig:a location (pose ?actual-object-pose)))))
;;            ?actual-object-pose))))))
;;
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
           ;;(if (grasping-retry>1)
           ;;trigger perception)
                       )))))))
   
