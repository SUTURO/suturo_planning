(defpackage :common-functions
  (:nicknames :comf)
  (:use :roslisp :cl :cram-designators)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun)
  (:export

   ;;high-level-plans
   #:move-hsr
   #:hsr-grasp
   #:grasp-hsr
   #:move-to-table
   #:move-to-shelf
   #:move-to-bucket

   ;;perception-functions
   #:get-confident-objects
   #:looking
   #:detecting
   
   ;;manipulation-functions
   #:place-object
   #:grasp-object
   #:create-place-list

   ;;knowledge-functions
   #:reachability-check
   #:reachability-check-grasp
   #:reachability-check-place
   #:prolog-object-goal-pose->pose-stamped

   ;;navigation-functions
   #:scan-object 
   #:move-to-poi-and-scan

   ;;safety-check
   #:execute-safety-check
   ))
