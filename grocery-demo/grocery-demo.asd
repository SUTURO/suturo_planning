(defsystem grocery-demo
  :depends-on (cram-language
               roslisp
               low-level-interfacing
               common-function)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ;;(:file "grocery-demo" :depends-on ("package"))
            ))))
