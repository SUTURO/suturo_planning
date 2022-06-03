(in-package :comf)
(defvar *point-behind-entrance-door* NIL) ;;TODO: set point
(defvar *point-behind-exit-door* NIL)     ;;TODO: set point
(defvar *reached-goal* (cram-language:make-fluent :name :reached-goal) NIL)

;;@author Tom-Eric Lehmkuhl
(defun execute-safety-check ()
  "Moving through the entrance door to the exit-door"
  (comf::with-hsr-process-modules
    (roslisp:ros-info (safety-check)
                      "Safety-check-plan started")
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

;;@author Tom-Eric Lehmkuhl, 2ndAuthor Luca Krohm
(defun check-goal-reached (nav-goal-pose-stamped)
  "Receives a nav goal `nav-goal-pose-stamped'. Checks whether the robot has approached the target point to within 10cm"
  (let ((distance 0))
    (loop do
      (roslisp::with-fields (translation) 
          (cl-tf::lookup-transform cram-tf::*transformer* 
                                   "map" "base_footprint")
        (setf distance (+ (abs (cl-tf:x translation))
                          (abs (cl-tf:y translation))
                          (abs (cl-tf:x nav-goal-pose-stamped))
                          (abs (cl-tf:y nav-goal-pose-stamped))))
        (cond
          ((> distance 0.2) (setf (cram-language:value *reached-goal*) T))))
      (cram-language:sleep 0.1))))
