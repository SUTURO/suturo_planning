(defsystem common-functions
  :depends-on (roslisp
               cram-language
               cl-tf
               cl-tf2
               actionlib
               geometry_msgs-msg
               tmc_msgs-msg
               low-level-interfacing
               cram-prolog
               cram-designators
               cram-process-modules
               cram-language-designator-support
               cram-executive)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on ("package"))
             (:file "perception-functions" :depends-on("package"))
             (:file "manipulation-functions" :depends-on("package"))
             (:file "navigation-functions" :depends-on("package")) 
             (:file "knowledge-functions" :depends-on("package"))
	     (:file "nlp-functions" :depends-on("package"))
	     (:file "high-level-plans" :depends-on("package")) 
))))
