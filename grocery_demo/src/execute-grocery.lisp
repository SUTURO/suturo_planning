(in-package :grocery)
(defparameter *perception-objects* NIL)
(defparameter *no-objects* NIL)
(defparameter *runing-fluent* (cram-language:make-fluent :name :objects) NIL)
(defparameter *next-object* NIL)
(defparameter *object-goal-pose* NIL)
(defparameter *grasp-object-result* NIL)
(defparameter *place-object-result* NIL)
(defparameter *grasping-retries* 0)

;;@author Torge Olliges, Tom-Eric Lehmkuhl
(defun execute-grocery()
    (comf::with-hsr-process-modules
    ;;get in park position
    (llif::call-take-pose-action 1)

    ;;TODO: uncomment when NLP works
    ;;waiting for start signal loop
    ;;(cram-language:pursue
    ;;    (cram-language:wait-for *runing-fluent*)
    ;;    (loop do
    ;;      (cram-language:sleep 0.1)))

    ;;move to table
    (cram-language:par 
        (llif::call-text-to-speech-action 
            "Hello, i am moving to the table now please step out of the way.")
        (cram-language:unwind-protect
            (comf::move-to-table T)))

    ;;perceiving the table
    (perceive-table)

    ;;move to table
    (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
    (comf::move-to-table NIL)

    ;;query for next object, grasp object, faiure handling for grasp
    (setf *next-object* (llif::prolog-next-object))
    ;;TODO: remove this replace with context
    (llif::call-text-to-speech-action "I am grasping the Object: ")
    (llif::call-text-to-speech-action 
        (first (split-sequence:split-sequence #\_ *next-object*)))
    (setf *grasp-object-result* (comf::grasp-object *next-object* 1))
    (grasp-handling)

    ;;move to shelf
    (cram-language:par
        (llif::call-text-to-speech-action 
            "Hello, i am moving to the shelf now please step out of the way.")
        (cram-language:unwind-protect
            (comf::move-to-shelf t)))  

    ;;perceive shelf
    (perceive-shelf "robocup_shelf_0")
    (perceive-shelf "robocup_shelf_1")
    (perceive-shelf "robocup_shelf_2")
    (llif::call-take-pose-action 1)
    (llif::call-text-to-speech-action "I am done perceiving the shelf now.")
  

    ;;TODO: uncomment when NLP works
    ;;(cram-language:pursue
    ;;(cram-language:wait-for *objects*)
    (loop do
        ;;placing
        (place-handling *next-object*)

          ;;back to base position
        (llif::call-take-pose-action 1)

        ;;move to table
        (llif::call-text-to-speech-action "I am getting into a position to grasp from.")
        (comf::move-to-table NIL)

        ;;query for next object
        (setf *next-object* 
            (llif::prolog-next-object))

        ;;grasp object
        (llif::call-text-to-speech-action "I am grasping the Object: ")
        (llif::call-text-to-speech-action 
            (first (split-sequence:split-sequence #\_ *next-object*)))
        (setf *grasp-object-result* (comf::grasp-object *next-object* 1))
        ;;faiure handling for grasp
        (grasp-handling)

        ;;query for knowledge if objects left
        (if (= (length 
            (llif::prolog-table-objects)) 0) 
            (set *no-objects* true)))))

;;@author Torge Olliges
;;Perceives a given shelf region (currently only shelf 0,1,2 due to robot capabilities)
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
   (setf *perception-objects* 
      (comf::get-confident-objects 
          (llif::call-robosherlock-object-pipeline 
              (vector shelf-region) 
              T)))
   (print *perception-objects*)
   (llif::insert-knowledge-objects *perception-objects*)
   (grocery::spawn-btr-objects *perception-objects*))

;;@author Torge Olliges
(defun perceive-table()
    (llif::call-text-to-speech-action "I am perceiving the table now.")
    (llif::call-take-pose-action 2)
    (setf *perception-objects* 
        (comf::get-confident-objects 
            (llif::call-robosherlock-object-pipeline 
                (vector "robocup_table") 
                T)))
    (llif::insert-knowledge-objects *perception-objects*)
    (grocery::spawn-btr-objects *perception-objects*)
    (llif::call-text-to-speech-action "I am done perceiving the table now.")
    (llif::call-take-pose-action 1))

;;deprecated
;;@author Torge Olliges
(defun grasp-handling ()
  ;;failure handling for grasping fail
        (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0)) 
          (llif::call-text-to-speech-action "I have successfully grasped the object")
          (progn (llif::call-text-to-speech-action "I have not grasped the object, looking for new object to grasp") 
                 ;;try to perceive again to get a better position
                 (comf::move-to-table T)
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

;;@author Torge Olliges
(defun grasp-with-failure-handling()
    (if (roslisp::with-fields (error_code) *grasp-object-result* (= error_code 0))
        (cpl:with-retry-counters ((grasping-retry 3))
            (cpl:with-failure-handling
                ;;TODO: find out which failure...
                (((or common-fail:low-level-failure 
                      cl::simple-error
                      cl::simple-type-error)
                (e)
                (progn 
                    (llif::call-text-to-speech-action "I have not grasped the object, looking for new object to grasp") 
                    ;;try to perceive again to get a better position
                    (comf::move-to-table T)
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
                    (setf *grasp-object-result* (comf::grasp-object *next-object* 1)))
                (cpl:do-retry grasping-retry
                    (roslisp:ros-warn 
                        (execute-grocery grasp-with-failure-handling)
                        "~%Failed to grasp the object~%")
                    (cpl:retry))
                (progn 
                    (roslisp:ros-warn (execute-grocery grasp-with-failure-handling) "~%No more retries~%")
                    (llif::call-text-to-speech-action "I am not able to grasp any object could you please put the object into my hand?")
                    ;;TODO: add the manipulation function for grasping where the gripper currently is here. if that fails shutdown?
                    )))
            (roslisp:ros-info (execute-grocery grasp-with-failure-handling) "Grasp failure handling successfull.")))))

;;@author Torge Olliges
(defun place-handling (object-name)
    ;;place position
    (llif::call-text-to-speech-action "I am getting into a position to place from.")
    (comf::move-to-shelf NIL)
    (llif::call-text-to-speech-action "I'm going to place the object in the shelf now.")
    ;;place object in shelf
    (setf *place-object-result* (comf::place-object object-name 1))
    (llif::call-text-to-speech-action "I have placed the object now."))


