(defsystem cleanup-demo
  :depends-on (cram-language
               roslisp
               common-functions
               low-level-interfacing)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "init" :depends-on ("package"))
            (:file "execute-cleanup" :depends-on("package"))
            ))))
