(defsystem suturo-real-hsr-pm
  :depends-on (roslisp-utilities ; for ros-init-function

               cram-process-modules
               cram-language
               cram-executive
               cram-designators
               cram-prolog
               cram-projection
               cram-occasions-events
               cram-utilities ; for EQUALIZE-LISTS-OF-LISTS-LENGTHS
               cram-hsrb-description

               cram-robokudo
               cram-giskard
               )
  
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "with-real-hsr-pm" :depends-on ("package"))))))
