(in-package :clean)

(defparameter *perception-objects* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *object-id* NIL)
(defparameter *next-object* NIL)
(defparameter *stamp-pose* NIL)
;;@Author Jan Schimpf
(defun execute-cleanup()
   (comf::with-hsr-process-modules

  (llif::call-text-to-speech-action "I am working now.")
;;  (shelf-scan)
;;  (table-scan)
;;  (transport)
  (point-of-interest-search)
  (point-of-interest-transport)
  )
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
  (let ((stamp-pose  (cl-tf::make-pose-stamped "map" 0 
                                               (cl-tf:make-3d-vector
                                               (nth 0 (nth 2 *object-goal-pose*))
                                               (nth 1 (nth 2 *object-goal-pose*))
                                               0) 
                                               (cl-tf::make-quaternion 0 0 0 1))))

  (comf::points-around-point 0.75 stamp-pose 8 NIL))
  (llif::call-text-to-speech-action *next-object*)
  ;; get into position to grasp

  ;; grasp the object
  (llif::call-text-to-speech-action "I'm going to grasp the object now.")
  (comf::grasp-object *next-object* 1)
  (llif::call-text-to-speech-action "I have grapsed the object")

  (comf::move-to-shelf NIL)
  ;;place object in shelf
  (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
  (comf::place-object *object-goal-pose* 1)
  (llif::call-text-to-speech-action "I have placed the object now.")

  ;;back to base position
  (llif::call-take-pose-action 1)
  ;; move into position to place object
  ;; place object
   )
  
 
  ;;(cram-language:pursue
      ;;(cram-language:wait-for *objects*)

(defun shelf-scan()
  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
  (comf::move-to-shelf t)
  ;;perceive shelf put objects into knowledge and bulletworld
  (llif::call-text-to-speech-action "I am perceiving the first shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_0"))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (clean::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am perceiving the second shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_1"))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (clean::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am perceiving the third shelf now.")
  (llif::call-take-pose-action 3)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_2"))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (clean::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am done perceiving the shelf now.")
  ;;back to base position
  (llif::call-take-pose-action 1)

  )

(defun table-scan()
  ;;move to table
  (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way, you fucker.")
  (comf::move-to-table t)
  
  ;;perceiving the table
  (llif::call-text-to-speech-action "I am perceiving the table now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
  (llif::insert-knowledge-objects *perception-objects*)
  (clean::spawn-btr-objects *perception-objects*)
  (llif::call-text-to-speech-action "I am done perceiving the table now.")
  (llif::call-take-pose-action 1)
  )

(defun transport()
        (loop do
        ;;move to table
        (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
        (comf::move-to-table NIL)

        ;;query for next object
        (setf *next-object* (llif::prolog-next-object))
        (llif::call-text-to-speech-action *next-object*)
        ;;grasp object
        (llif::call-text-to-speech-action "I'm going to grasp the object now.")
        (comf::grasp-object *next-object* 1)
        (llif::call-text-to-speech-action "I have grapsed the object")

        ;;move to shelf
        (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way, you fucker.")
        (comf::move-to-shelf)

        ;;check for correct position depending on other objects in shelf
        (setf *object-goal-pose* (llif::prolog-object-goal-pose *next-object*))

        ;;place object in shelf
        (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
        (comf::place-object *object-goal-pose* 1)
        (llif::call-text-to-speech-action "I have placed the object now.")

        ;;back to base position
        (llif::call-take-pose-action 1));;)

  )
  
