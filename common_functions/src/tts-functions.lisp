(in-package :comf)


(defun announce-movement-to-surface (time surface)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "move")
        (list "goal_surface_id" surface))))))

(defun announce-movement-to-room (time room))

(defun announce-movement (time)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "move"))))))

(defun announce-grasp-action (time object)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "grasp")
        (list "object_id" object))))))

(defun announce-place-action (time object)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "place")
        (list "object_id" object)
        (list "goal_surface_id" (llif::prolog-object-goal object)))))))

(defun announce-plan-start (plan)
  (llif::call-text-to-speech-action
   (llif::call-nlg-action-simple "starting" plan)))

(defun announce-perceive-action-surface (time surface)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "perceive")
        (list "goal_surface_id" surface))))))

(defun announce-perceive-action (time)
  (llif::call-text-to-speech-action
     (llif::get-string-from-nlg-result
      (llif::call-nlg-action-with-list
       (list
        (list "time" time)
        (list "action" "perceive"))))))
