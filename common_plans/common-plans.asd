(defsystem common-plans
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
             (:file "init" :depends-on ("package"))
             (:file "designators" :depends-on ("package"))
             ;;(:file "plans" :depends-on ("package"))
             (:file "utils" :depends-on ("package"))
             (:file "high-level" :depends-on("package"))
	     (:file "next-object" :depends-on("package"))   

))))
