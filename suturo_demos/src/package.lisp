


(defpackage :suturo-demos
  (:nicknames :su-demos)
  ;;(:use #:common-lisp :roslisp :cpl)
  (:use #:common-lisp #:cram-prolog #:cram-designators #:cram-executive)
  (:export
   #:cleanup-demo

   #:storing-groceries-demo

   #:set-the-table-demo

   #:clean-the-table-demo

   ;; Make sure that exported functions are always working and up to date
   ;; Functions that are not exported may be WIP
   
))
