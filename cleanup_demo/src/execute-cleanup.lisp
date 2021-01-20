(in-package :clean)

(defparameter *perception-objects* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *object-id* NIL)
(defparameter *next-object* NIL)
(defparameter *stamp-pose* NIL)

(defparameter *no-objects* (cram-language:make-fluent :name :objects) NIL)
(defparameter *grasp-object-result* NIL)
(defparameter *place-object-result* NIL)
(defparameter *grasping-retries* 0)

;;@author Jan Schimpf
;; execute the clean up plan
;; The plan start with percieving the table, grasps them and then places them in the bucked,
;; stops doing so when there are no more objects on the table
;; and then goes on to looking for objects on the floor and transports them to the bucked
(defun execute-cleanup()
    (comf::with-hsr-process-modules
      (llif::call-text-to-speech-action "I am working now.") ;;replace with NLG command ["starting","clean up"]
      ;;starts the table section for more info look at the functions
      (perceive-table)
      (transport)

      ;;finished with the table and starts on the floor and this loops until there are no more objects to transport
      (loop do
          (point-of-interest-search)
          (setf *next-object* (llif::prolog-next-object))
          (when (eq *next-object* 1) (return nil)) 
          (point-of-interest-transport))
      (llif::call-text-to-speech-action "I finished cleaning up the room")));;replace with NLG command

;;mostly copied from execute-grocery
;; Looks at the table, by first moving to the table then positioning so the robot can get a better picture,
;; which then are inserted into knowledge after which a position is taken so the robot can start transporting the objects
(defun perceive-table()
    ;;move to table
    (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way") ;;replace with NLG command [[action, "move"],[start_surface_id,"table"]]

    (comf::move-to-table t)
    ;;perceive-table
    (llif::call-text-to-speech-action "I am perceiving the table now.") ;;replace with NLG command  [[action, "percieve"],[start_surface_id,"table"]]
    (llif::call-take-pose-action 2)
    (setf *perception-objects*  (llif::call-robosherlock-object-pipeline (vector "table") t))
    (llif::insert-knowledge-objects *perception-objects*)
    (clean::spawn-btr-objects *perception-objects*)
    (llif::call-text-to-speech-action "I am done perceiving the table now.") ;;replace with NLG command  [[action, "move"],[start_surface_id,"table"]] ... not sure how to say that we finished this
    (llif::call-take-pose-action 1))

;;mostly copied from execute-grocery
;; This part takes care of transporting the objects from the table to the bucked.
;; For that it loops until it has no more objects to transport.
;; First knowledge is queried for the next-oobject and if there is none we end the loop.
;; If there is one the grasped after that the robot moves to the bucked and then place the object
;; above the bucked as it can't place them inside the bucked.
(defun transport()
    (loop do     
        ;;back to base position
        (llif::call-take-pose-action 1)

        ;;move to table
        (llif::call-text-to-speech-action "I am getting into a position to grasp from.") ;;replace with NLG command  [[action, "move"],[action,"grasp"]]

        ;;query for next object
        (setf *next-object* (llif::prolog-next-object))
      
        (when (eq *next-object* 1) (return) )

        (comf::move-to-table NIL)

        ;;grasp object
        (llif::call-text-to-speech-action "I am grasping the Object: ") ;;replace with NLG command  [[action, "grasp"],[object_id, *nect-object*]]
        (llif::call-text-to-speech-action (first (split-sequence:split-sequence #\_ *next-object*))) ;;replace with NLG command
        (setf *grasp-object-result* (comf::grasp-object *next-object* 1)) 

        ;;faiure handling for grasp
        (grasp-handling)

        ;;place position
        (llif::call-text-to-speech-action "I am getting into a position to place from.") ;;replace with NLG command  [[action, "move"],[action,"place"]]

        (comf::move-to-bucket)
        ;;place object in shelf
        (llif::call-text-to-speech-action  "I'm going to place the object in the bucket now.") ;;replace with NLG command  [[action,"place"],[object_id, *next-object*],[goal,surface_id,bucket]]

        (multiple-value-bind (a b) (llif::prolog-object-goal-pose *next-object*)
                                   (llif:call-text-to-speech-action b))
        (setf *place-object-result* (comf::place-object *next-object* 1))

        (llif::call-text-to-speech-action "I have placed the object now.") ;;replace with NLG command

        (llif::call-take-pose-action 1)))

;;copied from execute-grocery
(defun grasp-handling()
  ;;failure handling for grasping fail
        (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
          (llif::call-text-to-speech-action "I have successfully grasped the object") ;;replace with NLG command
          (progn (llif::call-text-to-speech-action "I have not grasped the object, looking for new object to grasp") ;;replace with NLG command
                 ;;try to perceive again to get a better position
                 (comf::move-to-table t)
                 (llif::prolog-forget-table-objects)
                 (btr-utils:kill-all-objects)
                 (perceive-table)
                 (llif::call-take-pose-action 1)
                 ;;Grasp again
                 (llif::call-text-to-speech-action "I am getting into a position to grasp from.") ;;replace with NLG command  [[action, "move"],[action,"place"]]
                 (comf::move-to-table NIL)
                 (setf *next-object* (llif::prolog-next-object))
                 (llif::call-text-to-speech-action "I am grasping the Object: ") ;;replace with NLG command  [[action,"place"],[object_id, *next-object*]]
                 (llif::call-text-to-speech-action (first (split-sequence:split-sequence #\_ *next-object*)))
                 (setf *grasp-object-result* (comf::grasp-object *next-object* 1))
                 ;;If it doesn't work again just stop trying
                 (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
                     (llif::call-text-to-speech-action "I have grapsed the object") ;;replace with NLG command
                     (if (< *grasping-retries* 3)
                        (grasp-handling)
                        (progn 
                          (llif::call-text-to-speech-action "I am not able to grasp any object could you please put the object into my hand?"))))))) ;;replace with NLG command


;; @author Jan Schimpf
;; goes to detected point of interest to scan them for objects, filters the objects,
;; then inserts them into the knowledge base and bulletworld and positions toya back into a neutral pose
(defun point-of-interest-search()
    (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command
    ;;drive to poi
    (comf::move-hsr  (cl-tf::make-pose-stamped "map" 0.0
                             (cl-tf::make-3d-vector 0.834 2.802 0)
                             (cl-tf::euler->quaternion :ax 0 :ay 0 :az 0 )))

    (comf::move-to-poi) 

    (llif::call-text-to-speech-action "I am perceiving the position now.") ;;replace with NLG command
    (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
    (llif::insert-knowledge-objects *perception-objects*)
    (clean::spawn-btr-objects *perception-objects*)
    ;;percieve -> filter -> insert into knowledge
    (llif::call-take-pose-action 1))


;;@author Jan Schimpf; Philipp Klein
;; Grasps the object and places it in the goal area (currently sill the shelf)
(defun point-of-interest-transport()
    (setf *next-object* (llif::prolog-next-object))

    (setf *object-goal-pose* (llif::prolog-object-pose *next-object*))
  
    ;; make sure we are in a neutral position
    (llif::call-text-to-speech-action "I'm turning towards the object, to grasp it.") ;;replace with NLG command  [[action, "percieve"],[start_surface_id,"floor"]]
    (llif::call-take-pose-action 1)
  
    ;; turn to face the object
    (roslisp::with-fields (translation rotation)
        (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
        (llif::call-nav-action-ps (cl-tf::make-pose-stamped "map" 0 translation
                                   (cl-tf::q* rotation
                                   (cl-tf::euler->quaternion :ax 0 :ay 0 :az -1.57)))))
  
    (llif::call-text-to-speech-action *next-object*) ;;replace with NLG command  [[action,"grasp"],[object_id, *next-object*]]
    ;; grasp the object from the floor

    (hsr-failure-handling-grasp)
  
    ;;move to bucket
    (comf::move-to-bucket)
  
    ;;place object in bucket
    (llif::call-text-to-speech-action "I'm going to place the object in the bucket now.") ;;replace with NLG command  [[action,"place"],[object_id, *next-object*],[goal,surface_id,bucket]]

    (comf::place-object *next-object* 1)
    (llif::call-text-to-speech-action "I have placed the object now.") ;;replace with NLG command

    ;;back to base position
    (llif::call-take-pose-action 1))

;;@author Jan Schimpf
;;Failure handling for grasping from the floor much more basic then from the
;;one from the table as we currently don't have a way to forget items on the floor.
(defun hsr-failure-handling-grasp()
    (cpl:with-retry-counters ((grasping-retry 1))
    (cpl:with-failure-handling
        (((or common-fail:low-level-failure 
                cl::simple-error
         cl::simple-type-error)
        (e)
        (llif::call-text-to-speech-action "I have failed to grasp the object
        could you please put the object into my hand? could you please give me the object ") ;;replace with NLG command

        (cpl:do-retry grasping-retry
            (roslisp:ros-warn (grasp-fail) "~%Failed to grasp the object~%")
            (cpl:retry))

        (roslisp:ros-warn (going-demo movement-fail)
                           "~%No more retries~%")))
  
        ;; grasp the object
        (llif::call-text-to-speech-action "I'm going to grasp the object now.") ;;replace with NLG command
        (comf::grasp-object *next-object* 1)
        (llif::call-text-to-speech-action "I have grapsed the object")))) ;;replace with NLG command
  


  
