(defsystem low-level-interfacing
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "manipulation-client" :depends-on ("package"))
            (:file "navigation-action" :depends-on ("package"))
	    (:file "perception-trigger-client" :depends-on ("package")))))
