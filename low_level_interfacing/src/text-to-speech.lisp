;; The text to speech client the HSR comes with
(in-package :llif)

(defvar *text-to-speech-publisher* nil)
(defparameter *enable-speech* nil)

(defvar *text-to-speech-action-client* nil)

(defun init-text-to-speech-action-client ()
  "Initializes the text to speech action client."
  (format t "Not doing this~%"))

(defun get-text-to-speech-action-client ()
  "Returns the current text to speech client. If none exists, one is created."
  (when (null *text-to-speech-action-client*)
    (init-text-to-speech-action-client))
  *text-to-speech-action-client*)

(defun make-text-action-goal (text)
  "Receives a text `text'. Create a text-to-speech action goal with `text'."
  (format t "Text:~%"))

;; used in cleanup
(defun call-text-to-speech-action (text)
  "Receives a text `text'. Calls the text to speech action to perform  `text'."
  (when *enable-speech*
    (multiple-value-bind (result status)
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-text-to-speech-action-client)
           (make-text-action-goal text)))
      (roslisp:ros-info (text-to-speech-action-client)
                        "Text to speech action finished.")
      (values result status)
      result))
  ;; print text on terminal, if speech is disabled (in simulator)
  (unless *enable-speech*
    (roslisp:ros-info (text-to-speech-action-client)
                      "~A"
                      text)))
