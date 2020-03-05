(in-package :grocery)
(defparameter *perception-objects* NIL)
;;(defparameter *running* NIL)
(defparameter *no-objects* (cram-language:make-fluent :name :objects) NIL)
(defparameter *next-object* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *grasp-object-result* NIL)

;;@author Torge Olliges, Tom-Eric Lehmkuhl
(defun execute-grocery()
  ;;get in park position
  (llif::call-take-pose-action 1)

  ;;TODO: uncomment when NLP works
  ;;waiting for start signal loop
  ;;(cram-language:pursue
  ;;    (cram-language:wait-for *state-fluent*)
  ;;    (loop do
  ;;      (cram-language:sleep 0.1)))

  ;;move to shelf
  (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way, you fucker.")
  (comf::move-to-shelf t)
  ;;perceive shelf put objects into knowledge and bulletworld
  (llif::call-text-to-speech-action "I am perceiving the first shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_shelf_0") t))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  ;;TODO: try this some time
  ;;(comf::scan-shelf "robocup_shelf_0")

  ;; (llif::call-text-to-speech-action "I have found: ")
  ;; (llif::call-text-to-speech-action (length *perception-objects*))
  ;; (llif::call-text-to-speech-action " Object in shelf 0 ")

  (llif::call-text-to-speech-action "I am perceiving the second shelf now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_shelf_1") t))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  ;; (llif::call-text-to-speech-action "I have found: ")
  ;; (llif::call-text-to-speech-action (length *perception-objects*))
  ;; (llif::call-text-to-speech-action " Object in shelf 1 ")

  (llif::call-text-to-speech-action "I am perceiving the third shelf now.")
  (llif::call-take-pose-action 3)
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_shelf_2") t))
  (print *perception-objects*)
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)

  ;; (llif::call-text-to-speech-action "I have found: ")
  ;; (llif::call-text-to-speech-action (length *perception-objects*))
  ;; (llif::call-text-to-speech-action " Object in shelf 2 ")

  (llif::call-text-to-speech-action "I am done perceiving the shelf now.")
  ;;back to base position
  (llif::call-take-pose-action 1)

  ;;move to table
  (llif::call-text-to-speech-action "Hello, i am moving to the table now please step out of the way, you fucker.")
  (comf::move-to-table t)
  
  ;;perceiving the table
  (llif::call-text-to-speech-action "I am perceiving the table now.")
  (llif::call-take-pose-action 2)
  (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
  (llif::insert-knowledge-objects *perception-objects*)
  (grocery::spawn-btr-objects *perception-objects*)
  (llif::call-text-to-speech-action "I am done perceiving the table now.")
  (llif::call-take-pose-action 1)

  ;;TODO: uncomment when NLP works
  ;;(cram-language:pursue
      ;;(cram-language:wait-for *objects*)
      (loop do
        ;;move to table
        (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
        (comf::move-to-table NIL)

        ;;query for next object
        (setf *next-object* (llif::prolog-next-object))


        ;;grasp object
        (llif::call-text-to-speech-action "I am grasping the Object: ")
        (llif::call-text-to-speech-action (first (split-sequence:split-sequence #\_ *next-object*)))
        (setf *grasp-object-result* (comf::grasp-object *next-object* 1))

        ;;failure handling for grasping fail
        (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
          (llif::call-text-to-speech-action "I have grapsed the object")
          (progn (llif::call-text-to-speech-action "I have not grapsed the object, looking for new object to grasp") 
                 ;;try to perceive again to get a better position
                 (comf::move-to-table t)
                 (llif::call-take-pose-action 2)
                 (setf *perception-objects* (llif::call-robosherlock-object-pipeline (vector "robocup_table") t))
                 (llif::insert-knowledge-objects *perception-objects*)
                 (llif::call-take-pose-action 1)
                 (grocery::spawn-btr-objects *perception-objects*)
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
                     (progn (llif::call-text-to-speech-action "Can't grasp anything i will stop now.") (return)))))

        

        ;;move to shelf
        (llif::call-text-to-speech-action "Hello, i am moving to the shelf now please step out of the way, you fucker.")
        (comf::move-to-shelf NIL)

        ;;get and check goal position
        (setf *object-goal-pose* (llif::prolog-object-goal-pose *next-object*))

        ;;(if (= *object-goal-pose* 1) 
        ;;   (progn (llif::call-text-to-speech-action "No object goal pose received.")
        ;;          (return)
        ;;   ) 
        ;;   (llif::call-text-to-speech-action "Received object goal pose."))

        ;;place object in shelf
        (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
        (comf::place-object *object-goal-pose* 1)
        (llif::call-text-to-speech-action "I have placed the object now.")

        ;;back to base position
        (llif::call-take-pose-action 1));;)

        ;;query for knowledge if objects left
        ;;(if (= (length (llif::prolog-table-objects)) 0) (set *no-objects* true)
)
