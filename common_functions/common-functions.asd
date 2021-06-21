(defsystem common-functions
  :depends-on (roslisp
               cram-language
               cl-tf
               cl-tf2
               actionlib
               geometry_msgs-msg
               ;;tmc_msgs-msg
               cram-prolog
               cram-designators
               cram-process-modules
               cram-language-designator-support
               cram-executive
               low-level-interfacing)
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
             ;;select process modules and process modules depend on each other
             (:file "select-process-modules" :depends-on("package"))
	     (:file "process-modules" :depends-on("package"))
             ;;high level plans depends on every other file in here
             (:file "high-level-plans" :depends-on("package"))
	     (:file "safety-check" :depends-on("package"))
	     (:file "tts-functions" :depends-on("package"))))))
