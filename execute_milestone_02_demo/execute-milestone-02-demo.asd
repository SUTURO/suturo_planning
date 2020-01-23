(defsystem execute-milestone-02-demo
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "execute-miltestone-02-demo" :depends-on ("package"))
            (:file "demo-poses" :depends-on ("package"))   
            ))))
