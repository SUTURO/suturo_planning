(defsystem cleanup-demo
  :depends-on (cram-language
               roslisp
               common-function
               low-level-interfacing)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "cleanup-demo" :depends-on ("package"))
            (:file "init" :depends-on ("package"))
            ))))
