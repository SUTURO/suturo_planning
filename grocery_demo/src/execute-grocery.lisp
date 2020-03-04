(in-package :grocery)
(defparameter *perception-objects* NIL)
;;(defparameter *running* NIL)
(defparameter *no-objects* (cram-language:make-fluent :name :objects) NIL)
(defparameter *next-object* NIL)
(defparameter *object-goal-pose* NIL)

;;@author Torge Olliges, Tom-Eric Lehmkuhl
(defun execute-grocery()
  ;;waiting for start signal loop
  ;;(cram-language:pursue
  ;;    (cram-language:wait-for *state-fluent*)
  ;;    (loop do
  ;;      (cram-language:sleep 0.1)))

  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
  (comf::move-to-shelf)
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

  (llif::call-text-to-speech-action "I am done perceiving the shelf now.")
  ;;back to base position
  (llif::call-take-pose-action 1)

  ;;move to table
  (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way.")
  (comf::move-to-table)

  ;;perceiving the table
  (llif::call-text-to-speech-action "I am perceiving the table now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-pipeline "robocup_table"))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)
  (llif::call-text-to-speech-action "I am done perceiving the table now.")

  ;;(cram-language:pursue
      ;;(cram-language:wait-for *objects*)
      (loop do
        ;;move to table
        (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
        (comf::move-to-table)

        ;;query for next object
        (setf *next-object* (llif::prolog-next-object))
        ;;grasp object
        (llif::call-text-to-speech-action "I'm going to grasp the object now.")
        (comf::grasp-hsr *next-object* 1)
        (llif::call-text-to-speech-action "I have grapsed the object")

        ;;move to shelf
        (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way.")
        (comf::move-to-shelf)

        ;;check for correct position depending on other objects in shelf
        (setf *object-goal-pose* (llif::prolog-object-goal-pose *next-object*))

        ;;place object in shelf
        (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
        (comf::place-object *object-goal-pose* 1)
        (llif::call-text-to-speech-action "I have placed the object now.")

        ;;back to base position
        (llif::call-take-pose-action 1));;)

        ;;query for knowledge if objects left
        ;;(if (= (length (llif::prolog-table-objects)) 0) (set *no-objects* true)
)
