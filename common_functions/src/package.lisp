(defpackage :common-functions
  (:nicknames :comf)
  (:use :roslisp :cl :cram-designators)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun)
  (:export

   ;;high-level-plans.lisp
   #:move-hsr
   #:hsr-grasp
   #:grasp-hsr
   

   ;;perception-functions.lisp
   #:get-confident-objects
   #:looking
   #:detecting
   
   ;;manipulation-functions.lisp
   #:place-object
   #:grasp-object
   #:create-place-list

   ;;knowledge-functions.lisp
   #:next-object	

   ;;navigation-functions.lisp
   #:scan-object 
   #:move-to-poi-and-scan

   ;;safety-check.lisp
   #:execute-safety-check
   ))
