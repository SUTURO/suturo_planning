(defsystem execute-milestone1-demo
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "execute-miltestone1-demo" :depends-on ("package")) 
            ))))
