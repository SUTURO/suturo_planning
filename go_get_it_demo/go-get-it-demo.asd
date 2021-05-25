(defsystem go-get-it-demo
  :depends-on (cram-language
               roslisp
	             cram-hsrb-pick-demo
               cram-urdf-bringup
               common-functions
               low-level-interfacing
               nlp_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "init" :depends-on ("package"))
             ;;execute cleanup depends on init
             (:file "execute-go-get-it" :depends-on ("package"))
             (:file "go-get-it-bw" :depends-on ("package"))))))
