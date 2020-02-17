(defsystem cleanup-demo
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "cleanup-demo" :depends-on ("package"))
            ))))
