(defsystem go-get-it-demo
  :depends-on (cram-language
               roslisp
	             cram-hsrb-pick-demo
               cram-urdf-bringup
               common-functions
               low-level-interfacing)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "init" :depends-on ("package"))
            ;;execute cleanup depends on init
            (:file "execute-go-fetch" :depends-on ("package"))
            ))))
