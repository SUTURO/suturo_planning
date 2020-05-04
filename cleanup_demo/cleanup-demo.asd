(defsystem cleanup-demo
  :depends-on (cram-language
               roslisp
	       cram-hsrb-pick-demo
               common-functions
               low-level-interfacing)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "init" :depends-on ("package"))
            ;;execute cleanup depends on init
            (:file "execute-cleanup" :depends-on ("package"))
            (:file "cleanup-bw" :depends-on ("package"))
            ))))
