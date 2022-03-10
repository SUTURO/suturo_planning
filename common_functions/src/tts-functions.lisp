(in-package :comf)

;; used in cleanup
;;@author Torge Olliges
(defun announce-plan-start (plan)
  (llif::call-text-to-speech-action
   (llif::call-nlg-action-simple "starting" plan)))
