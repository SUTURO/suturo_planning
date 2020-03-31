(in-package :clean)

(defparameter *perception-objects* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *object-id* NIL)
(defparameter *next-object* NIL)
(defparameter *stamp-pose* NIL)
(defparameter *grasping-retries* 0)

(defparameter *no-objects* (cram-language:make-fluent :name :objects) NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *grasp-object-result* NIL)
(defparameter *place-object-result* NIL)
(defparameter *grasping-retries* 0)

;;@Author Jan Schimpf
(defun execute-cleanup()
  (comf::with-hsr-process-modules

    (llif::call-text-to-speech-action "I am working now.")
    (shelf-scan)
    (perceive-table)
    (transport)
    (loop do
    (point-of-interest-search)
    (point-of-interest-transport)
    ))
  )

(defun point-of-interest-search()
  (llif::call-text-to-speech-action "I have found a point of interest to search.")
  ;;drive to poi
  (comf::move-to-poi)

  (llif::call-text-to-speech-action "I am perceiving the position now.")
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
  (llif::insert-knowledge-objects *perception-objects*)
  (clean::spawn-btr-objects *perception-objects*)
  ;;percieve -> filter -> insert into knowledge
  (llif::call-take-pose-action 1)

  )

(defun point-of-interest-transport()
  
   ;; get next-object
  (setf *next-object* (llif::prolog-next-object))
  (setf *object-goal-pose* (llif::prolog-object-pose *next-object*))

  (llif::call-text-to-speech-action "I'm turning towards the object, to grasp it.")
  (llif::call-take-pose-action 1)
  ;; turn to face the object
  ;;@author phillip
  (roslisp::with-fields (translation rotation) (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
    (llif::call-nav-action-ps (cl-tf::make-pose-stamped
                               "map" 0 translation
                               (cl-tf::q* rotation (cl-tf::euler->quaternion :ax 0 :ay 0 :az -1.57) )))
    )
  (llif::call-text-to-speech-action *next-object*)
  ;; get into position to grasp

  (hsr-failure-handling-grasp)
  
  ;;move to shelf
  (comf::move-to-shelf NIL)
  
  ;;place object in shelf
  (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
  (comf::place-object *next-object* 1)
  (llif::call-text-to-speech-action "I have placed the object now.")

  ;;back to base position
  (llif::call-take-pose-action 1)
   )
  
(defun hsr-failure-handling-grasp()
   (cpl:with-retry-counters ((grasping-retry 1))
   (cpl:with-failure-handling
      (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
       (e)
       (llif::call-text-to-speech-action "I have failed to grasp the object
could you please put the object into my hand? could you please give me the object ")

       (cpl:do-retry grasping-retry
         (roslisp:ros-warn (grasp-fail)
                                 "~%Failed to grasp the object~%")
         (cpl:retry))

         (roslisp:ros-warn (going-demo movement-fail)
                           "~%No more retries~%")))
  

  ;; grasp the object
  (llif::call-text-to-speech-action "I'm going to grasp the object now.")
  (comf::grasp-object *next-object* 1)
  (llif::call-text-to-speech-action "I have grapsed the object"))))
  



(defun shelf-scan()
  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
  (comf::move-to-shelf t)
  
  ;;perceive shelf
  (perceive-shelf "robocup_shelf_0")
  (perceive-shelf "robocup_shelf_1")
  (perceive-shelf "robocup_shelf_2")
  (llif::call-take-pose-action 1)
  (llif::call-text-to-speech-action "I am done perceiving the shelf now.")
  )

;;copied from execute grocery
(defun perceive-shelf(shelf-region)
  (case shelf-region 
     ("robocup_shelf_0" 
       (progn 
         (llif::call-text-to-speech-action "I am perceiving shelf zero now.")
         (llif::call-take-pose-action 2)))
     ("robocup_shelf_1" 
       (progn 
         (llif::call-text-to-speech-action "I am perceiving shelf one now.")
         (llif::call-take-pose-action 2)))
     ("robocup_shelf_2" 
       (progn 
         (llif::call-text-to-speech-action "I am perceiving shelf two now.")
         (llif::call-take-pose-action 3))))
   (setf *perception-objects* (comf::get-confident-objects (llif::call-robosherlock-object-pipeline (vector shelf-region) t)))
   (print *perception-objects*)
   (llif::insert-knowledge-objects *perception-objects*)
   (grocery::spawn-btr-objects *perception-objects*)
)

(defun perceive-table()
  ;;move to table
  (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way")
  (comf::move-to-table t)
  ;;perceive-table
  (llif::call-text-to-speech-action "I am perceiving the table now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (comf::get-confident-objects (llif::call-robosherlock-object-pipeline (vector "robocup_table") t)))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)
  (llif::call-text-to-speech-action "I am done perceiving the table now.")
  (llif::call-take-pose-action 1)
)

;;mostly copied from execute-grocery
(defun transport()

      (loop do     
      ;;back to base position
      (llif::call-take-pose-action 1)

      ;;move to table
      (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
      (comf::move-to-table NIL)

      ;;query for next object
      (setf *next-object* (llif::prolog-next-object))

      ;;grasp object
      (llif::call-text-to-speech-action "I am grasping the Object: ")
      (llif::call-text-to-speech-action (first (split-sequence:split-sequence #\_ *next-object*)))
      (setf *grasp-object-result* (comf::grasp-object *next-object* 1))

      ;;faiure handling for grasp
      (grasp-handling)

      ;;move to shelf
      (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
      (comf::move-to-shelf Nil)

      ;;place position
      (llif::call-text-to-speech-action "I am getting into a position to place from.")
      (comf::move-to-shelf NIL)
      ;;place object in shelf
      (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
      (setf *place-object-result* (comf::place-object *next-object* 1))

      (llif::call-text-to-speech-action "I have placed the object now.")

      (llif::call-take-pose-action 1)

            ))

;;copied from execute-grocery
(defun grasp-handling()
  ;;failure handling for grasping fail
        (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
          (llif::call-text-to-speech-action "I have successfully grasped the object")
          (progn (llif::call-text-to-speech-action "I have not grasped the object, looking for new object to grasp") 
                 ;;try to perceive again to get a better position
                 (comf::move-to-table t)
                 (llif::prolog-forget-table-objects)
                 (btr-utils:kill-all-objects)
                 (perceive-table)
                 (llif::call-take-pose-action 1)
                 ;;Grasp again
                 (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
                 (comf::move-to-table NIL)
                 (setf *next-object* (llif::prolog-next-object))
                 (llif::call-text-to-speech-action "I am grasping the Object: ")
                 (llif::call-text-to-speech-action (first (split-sequence:split-sequence #\_ *next-object*)))
                 (setf *grasp-object-result* (comf::grasp-object *next-object* 1))
                 ;;If it doesn't work again just stop trying
                 (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
                     (llif::call-text-to-speech-action "I have grapsed the object") 
                     (if (< *grasping-retries* 3)
                        (grasp-handling)
                        (progn 
                          (llif::call-text-to-speech-action "I am not able to grasp any object could you please put the object into my hand?")))))))
  
