(in-package :comf)
(defvar *point-behind-entrance-door* NIL) ;;TODO: set point
(defvar *point-behind-exit-door* NIL)     ;;TODO: set point
(defvar *reached-goal* (cram-language:make-fluent :name :reached-goal) NIL)
;;TODO: fix indentation similar to high-level/grocery-execute
;;@author Tom-Eric Lehmkuhl
(defun execute-safety-check ()
  "safety-check-plan: moving through the entrance door to the exit-door."

  (comf::with-hsr-process-modules
    (roslisp:ros-info (safety-check) "Safety-check-plan started")
    (llif::call-text-to-speech-action "Please open the door. When the door is
                                       open, i will move into the room.")
    (cram-language:par
      (check-goal-reached *point-behind-entrance-door*)
      (cram-language:pursue
        ;;TODO: Recognizing that the door is open
            (cram-language:wait-for *reached-goal*)  
            (loop do
                (cram-language:unwind-protect
                    (comf::move-hsr *point-behind-entrance-door*)))))

    (cram-language:par
        (llif::call-text-to-speech-action "I am moving to the exit-door now.")
        (cram-language:unwind-protect
            (comf::move-hsr *point-behind-exit-door*)))))

;;@author Tom-Eric Lehmkuhl
(defun check-goal-reached (nav-goal-pose-stamped)
  "checks if the goal is reached.This is done by checking whether the robot
   has approached the target point to within 10cm."
    (defvar *distance* 0)
    (loop do
        (roslisp::with-fields (translation) 
            (cl-tf::lookup-transform cram-tf::*transformer* 
                                     "map" "base_footprint")
          (setf *distance* (+ (abs (cl-tf:x translation))
                                (+ (abs (cl-tf:y translation))
                                   (+ (abs (cl-tf:x nav-goal-pose-stamped))
                                      (abs (cl-tf:y nav-goal-pose-stamped))))))
            (if (> *distance* 0.2) 
                (setf (cram-language:value *reached-goal*) T)))
        (cram-language:sleep 0.1)))
    
