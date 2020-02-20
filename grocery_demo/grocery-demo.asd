(defsystem grocery-demo
  :depends-on (cram-language
               roslisp
               low-level-interfacing
               common-functions)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "execute-grocery" :depends-on ("package"))
	    (:file "init" :depends-on("package"))
            ))))
