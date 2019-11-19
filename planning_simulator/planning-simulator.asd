(defsystem planning-simulator
  :depends-on (cram-language)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "simulator":depends-on ("package"))))))

