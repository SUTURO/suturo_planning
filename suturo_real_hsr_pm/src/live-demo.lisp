(in-package :su-real)

;; author Luca Krohm
(defun demo (demo)
  (defvar *plan* nil)
  (case demo
    (:clean (setf *plan* (list #'su-demos::clean-the-table-demo2)))
    (:groceries (setf *plan* (list #'su-demos::storing-groceries-demo2)))
    (:breakfast (setf *plan* (list #'su-demos::serve-breakfast-demo)))
    (:all (setf *plan* (list #'su-demos::storing-groceries-demo2
                                #'su-demos::serve-breakfast-demo
                                #'su-demos::clean-the-table-demo2)))
    (otherwise (roslisp:ros-error (run-demo)
                                  "Demo ~a is not a valid demo!"
                                  demo)))
   (with-hsr-process-modules
    (unwind-protect
         (progn
           (roslisp-utilities:startup-ros)
           (mapc #'funcall *plan*)
      (roslisp-utilities:shutdown-ros)))))
