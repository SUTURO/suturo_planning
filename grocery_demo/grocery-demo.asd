(defsystem grocery-demo
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
            ;;execute-grocery depends on init
            (:file "execute-grocery" :depends-on ("package"))
            (:file "grocery-bw" :depends-on ("package"))
            ))))
