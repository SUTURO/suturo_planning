(defsystem grocery-demo
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "grocery-demo" :depends-on ("package"))
            ))))
