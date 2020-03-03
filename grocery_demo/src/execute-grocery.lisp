(in-package :grocery)
(defparameter *perception-objects* NIL)
(defparameter *running* NIL)
(defparameter *next-object* NIL)

;;@author Torge Olliges
(defun execute-grocery()
  ;;waiting for start signal loop
  (cram::wait-for-fluent *running*)

  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
  ;;TODO: move-hsr is not parameterised
  (comf::move-hsr (llif::prolog-shelf-pose))
  ;;perceive shelf put objects into knowledge and bulletworld
  (llif::call-text-to-speech-action "I am perceiving the first shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_0"))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am perceiving the second shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_1"))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am perceiving the third shelf now.")
  (llif::call-take-pose-action 3)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_shelf_2"))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech "I am done perceiving the shelf now.")
  ;;back to base position
  (llif::call-take-pose-action 1)

  ;;move to table
  (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way.")
  ;;TODO: move-hsr is not parameterised
  (comf::move-hsr (llif::prolog-table-pose))

  ;;perceiving the table
  (llif::call-text-to-speech-action "I am perceiving the table now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_table"))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  (llif::call-text-to-speech-action "I am done perceiving the table now.")

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LOOP
  ;;move to table
  (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
  ;;TODO: move-hsr is not parameterised
  (comf::move-hsr (llif::prolog-table-pose))

  ;;query for next object
  (setf *next-object* (llif::prolog-next-object))
  ;;grasp object
  (comf::grasp-object *next-object* 1)

  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
  ;;TODO: move-hsr is not parameterised
  (comf::move-hsr (llif::prolog-shelf-pose))

  ;;check for correct position depending on other objects in shelf
  (stf *object-goal-pose* (llif::prolog-object-goal-pose *next-object*))

  ;;place object in shelf
  (comf::place-object *object-goal-pose*)

  ;;back to base position
  (llif::call-take-pose-action 1)

  ;;query for knowledge if objects left
  ;;(if (= (length (llif::prolog-table-objects)) 0) (;;continue loop) (;;stop loop))
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LOOP
  )