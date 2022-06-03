(in-package :comf)

;; used in cleanup
;;@author Torge Olliges
(defun announce-plan-start (plan)
  "Receives plan `plan'. Announces start of \variable{plan} via text-to-speech."
  (llif::call-text-to-speech-action
   (llif::call-nlg-action-simple "starting" plan)))
