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
(defparameter *graspmode* NIL)


(defun init-execute-cleanup()
  (init-interfaces)
  (execute-cleanup)
  )

;;@author Jan Schimpf
;; execute the clean up plan
;; The plan start with percieving the table, grasps them and then places them in the bucked,
;; stops doing so when there are no more objects on the table
;; and then goes on to looking for objects on the floor and transports them to the bucked


(defun execute-cleanup()
    ;;(init-interfaces)
    (comf::with-hsr-process-modules
      (comf::announce-plan-start "clean up")
      ;;starts the table section for more info look at the functions
      ;;move to table
      (comf::announce-movement-to-surface "future" "table")
      (comf::move-to-table t)

      (perceive-table)
      (transport)

        ;;finished with the table and starts on the floor and this loops until there are no more objects to transport
        (loop do
            (point-of-interest-search)
            (setf *next-object* (llif::prolog-next-object))
            (when (eq *next-object* 1) (return nil)) 
            (point-of-interest-transport))
      (loop do
        (point-of-interest-search-second-point)
            (setf *next-object* (llif::prolog-next-object))
            (when (eq *next-object* 1) (return nil)) 
        (point-of-interest-transport))
        (llif::call-text-to-speech-action "I finished cleaning up the room")));;replace with NLG command

;; mostly copied from execute-grocery
;; Looks at the table, by first moving to the table then positioning so the robot can get a better picture,
;; which then are inserted into knowledge after which a position is taken so the robot can start transporting the objects
(defun perceive-table()
    ;;perceive-table
    (comf::announce-perceive-action-surface "present" "table")

    (llif::call-take-pose-action 2)
    (setf *perception-objects*  (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
    (llif::insert-knowledge-objects *perception-objects*)
    (clean::spawn-btr-objects *perception-objects*)
    (comf::announce-perceive-action-surface "past" "table")
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

        ;;query for next object
        (setf *next-object* (llif::prolog-next-object))
        (when (eq *next-object* 1) (return))
        
        (comf::reachability-check-grasp *next-object* 1)

        (comf::announce-grasp-action  "future" *next-object*)

        (comf::move-to-table NIL)

        ;;grasp object
        (setf *graspmode* 1) ;;sets the graspmode should be replaced with the function from knowledge when that is finished
        (setf *grasp-object-result* (comf::grasp-object *next-object* *graspmode*))
        
        ;;faiure handling for grasp
        (grasp-handling)

        ;;place position
        (comf::announce-movement-to-surface "present" "bucket")

        (comf::move-to-bucket)
        ;;place object in shelf
        (comf::announce-place-action "future" *next-object*)

        
        (multiple-value-bind (a b) 
            (llif::prolog-object-goal-pose *next-object*)
            (llif:call-text-to-speech-action b))
        (setf *place-object-result* (comf::place-object *next-object* *graspmode*))

        (if (eq *place-object-result* 1)
            (comf::announce-place-action "past" *next-object*)
            (comf::announce-place-action "failed" *next-object*))
        (llif::call-take-pose-action 1)))

;;copied from execute-grocery
(defun grasp-handling()
;;failure handling for grasping fail
    (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
        (comf::announce-grasp-action "past" *next-object*)
        (progn 
            (llif::call-text-to-speech-action "I have not grasped the object, looking for new object to grasp") ;;replace with NLG command
            ;;try to perceive again to get a better position
            (comf::move-to-table t)
            (llif::prolog-forget-table-objects)
            (btr-utils:kill-all-objects)
            (perceive-table)
            (llif::call-take-pose-action 1)
            ;;Grasp again
            (comf::announce-movement "present")
            (comf::move-to-table NIL)
            (setf *next-object* (llif::prolog-next-object))
            (comf::announce-grasp-action "present" *next-object*)
            (setf *graspmode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
            (setf *grasp-object-result* (comf::grasp-object *next-object* *graspmode*))
            ;;If it doesn't work again just stop trying
            (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
                (comf::announce-grasp-action "past" *next-object*)
                (if (< *grasping-retries* 3)
                    (grasp-handling)
                    (comf::announce-grasp-action "failed" *next-object*)))))) ;;replace with NLG command


;; @author Jan Schimpf
;; goes to detected point of interest to scan them for objects, filters the objects,
;; then inserts them into the knowledge base and bulletworld and positions toya back into a neutral pose
(defun point-of-interest-search()
    (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command
    ;;drive to poi
    (comf::move-hsr  
        (cl-tf::make-pose-stamped "map" 0.0
            (cl-tf::make-3d-vector 1.45 -0.01 0)
            (cl-tf::euler->quaternion :ax 0 :ay 0 :az 0)))

    (comf::move-to-poi) 

    (comf::announce-perceive-action "future")
    (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
    ;;(comf:reachability-check *perception-objects*)
    (llif::insert-knowledge-objects *perception-objects*)
    ;;(comf:reachability-check (llif::prolog-next-graspable-objects))
    (clean::spawn-btr-objects *perception-objects*)
    ;;percieve -> filter -> insert into knowledge
    (llif::call-take-pose-action 1))

(defun point-of-interest-search-second-point()
    (llif::call-text-to-speech-action "I have found a point of interest to search.") ;;replace with NLG command
    ;;drive to poi
    (comf::move-hsr  
        (cl-tf::make-pose-stamped "map" 0.0
            (cl-tf::make-3d-vector 2.32 1.5 0)
            (cl-tf::euler->quaternion :ax 0 :ay 0 :az 2.5)))

    (comf::move-to-poi) 

    (comf::announce-perceive-action "future")
    (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_default") t))
    ;;(comf:reachability-check *perception-objects*)
    (llif::insert-knowledge-objects *perception-objects*)
    ;;(comf:reachability-check (llif::prolog-next-graspable-objects))
    (clean::spawn-btr-objects *perception-objects*)
    ;;percieve -> filter -> insert into knowledge
    (llif::call-take-pose-action 1))

;;@author Jan Schimpf; Philipp Klein
;; Grasps the object and places it in the goal area (currently sill the shelf)
(defun point-of-interest-transport()
    (setf *next-object* (llif::prolog-next-object))

    (setf *object-goal-pose* (llif::prolog-object-pose *next-object*))

    ;; make sure we are in a neutral position
    (comf::announce-grasp-action "future" *next-object*)
    (llif::call-take-pose-action 1)
    ;; turn to face the object
    ;;(roslisp::with-fields (translation rotation)
    ;;    (cl-tf::lookup-transform cram-tf::*transformer* "map" "base_footprint")
    ;;    (llif::call-nav-action-ps 
    ;;        (cl-tf::make-pose-stamped "map" 0 translation
    ;;            (cl-tf::q* rotation
    ;;            (cl-tf::euler->quaternion :ax 0 :ay 0 :az -1.57)))))
    
    ;; grasp the object from the floor
    (hsr-failure-handling-grasp)
    (comf::announce-movement  "future")
    ;;move to bucket
    (llif::call-nlg-action (list (llif::make-key-value-msg "action" "move") (llif::make-key-value-msg "goal_surface_id" "bucket")))
    (comf::move-to-bucket)

    ;;place object in bucket
    (comf::announce-place-action "present" *next-object*)

    (comf::place-object *next-object* *graspmode*)
    (comf::announce-place-action "past" *next-object*)
    ;;back to base position
    (llif::call-take-pose-action 1))

;;@author Jan Schimpf
;;Failure handling for grasping from the floor much more basic then from the
;;one from the table as we currently don't have a way to forget items on the floor.
(defun hsr-failure-handling-grasp()
    (cpl:with-retry-counters ((grasping-retry 1))
      (cpl:with-failure-handling
        (((or 
            common-fail:low-level-failure 
            cl::simple-error
            cl::simple-type-error)
             (e)
           (comf::announce-grasp-action "failed" *next-object*)
        (llif::call-text-to-speech-action "I have failed to grasp the object
        could you please put the object into my hand? could you please give me the object ") ;;replace with NLG command
        (cpl:do-retry grasping-retry
            (roslisp:ros-warn (grasp-fail) "~%Failed to grasp the object~%")
            (cpl:retry))

        (roslisp:ros-warn (going-demo movement-fail) "~%No more retries~%")))

        ;; grasp the object
        (comf::announce-grasp-action "future" *next-object*)
        (setf *graspmode* 1)  ;;sets the graspmode should be replaces with the function from knowledge when that is finished
        (comf::grasp-object *next-object* *graspmode*)
        (comf::announce-grasp-action "past" *next-object*)))) ;;replace with NLG command

