(defsystem low-level-interfacing
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "navigation-action" :depends-on ("package"))))))
