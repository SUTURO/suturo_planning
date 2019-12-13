(defsystem common-plans
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
            ))))
