(in-package :comf)
(defvar *point-behind-entrance-door* NIL) ;;TO-DO: set point
(defvar *point-behind-exit-door* NIL)     ;;TO-DO: set point
(defvar *reached-goal* (cram-language:make-fluent :name :reached-goal) NIL)
;;TODO: fix indentation similar to high-level/grocery-execute
;;@author Tom-Eric Lehmkuhl
(defun execute-safety-check ()
 "safety-check-plan: moving through the entrance door to the exit-door."
    (roslisp:ros-info (safety-check) "Safety-check-plan started")
    (llif::call-text-to-speech-action "Please open the door. When the door is open, 
                                        i will move into the room.")
    (cram-language:par
        (check-goal-reached *point-behind-entrance-door*)
        (cram-language:pursue
            (cram-language:wait-for *reached-goal*) ;;TO-DO: Recognizing that the door is open
            (loop do
                (cram-language:unwind-protect
                    (comf::move-hsr *point-behind-entrance-door*)))))

    (cram-language:par
        (llif::call-text-to-speech-action "I am moving to the exit-door now.")
        (cram-language:unwind-protect
            (comf::move-hsr *point-behind-exit-door*))))

;;@author Tom-Eric Lehmkuhl
(defun check-goal-reached (nav-goal-pose-stamped)
 "checks if the goal is reached."
    (loop do
        (roslisp::with-fields (translation) 
            (cl-tf::lookup-transform cram-tf::*transformer* 
                                                        "map" "base_footprint")
            (if (cl-tf::eq translation (cl-tf::origin nav-goal-pose-stamped)) 
                (setf (cram-language:value *reached-goal*) T)))
        (cram-language:sleep 0.1)))
    