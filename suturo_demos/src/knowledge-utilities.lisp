(in-package :su-demos)

(defmacro with-safe-prolog (&body body)
  "Receives body of LISP code `body'. Runs body with failurehandling."
  `(handler-case
       ,@body
     (simple-error ()
       (roslisp:ros-error (json-prolog-client)
                          "Json prolog client error. Check your query again."))
     (SB-KERNEL:CASE-FAILURE ()
       (roslisp:ros-error (json-prolog-client)
                          "Startup your rosnode first"))
     (ROSLISP::ROS-RPC-ERROR ()
       (roslisp:ros-error (json-prolog-client)
                          "Is the json_prolog server running?"))))


;; @author Luca Krohm
(defun match-prolog-symbol (symbol)
  "Adds a '?' to the symbol, since prolog takes the value of the variable 'result' in CALL-KNOWLEDGE, and does this too. For example: FRAME -> ?FRAME"
   (intern (concatenate 'string "?" (symbol-name symbol))))

;; @author Luca Krohm
(defun fix-prolog-string (prolog-symbol)
  (typecase prolog-symbol
    (symbol (let ((result (symbol-name prolog-symbol)))
              (subseq result 1 (- (length result) 1))))
    (list (mapcar #'fix-prolog-string prolog-symbol))
    (t prolog-symbol)))
    
