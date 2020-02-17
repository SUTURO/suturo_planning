(defpackage :common-functions
  (:nicknames :comf)
  (:use :roslisp :cl :cram-designators)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun)
  (:export

   ;;high-level-plans.lisp
   #:move-hsr
   #:hsr-grasp

   ;;perception-functions.lisp
   #:get-confident-objects

   ;;manipulation-functions.lisp
   #:place-object
   #:grasp-object

   ;;knowledge-functions.lisp
   #:next-object	

   ))