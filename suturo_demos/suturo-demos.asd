(defsystem suturo-demos
  :depends-on (roslisp-utilities ; for ros-init-function

	       actionlib
               actionlib_msgs-msg

               cl-transforms
               cl-transforms-stamped
               cl-tf
	       cl-tf2
	       cl-utils
               cram-tf
               cram-math

               cram-process-modules
               cram-language
               cram-executive
               cram-designators
               cram-prolog
               cram-projection
               cram-occasions-events
               cram-utilities ; for EQUALIZE-LISTS-OF-LISTS-LENGTHS
	     

               cram-common-failures
               cram-mobile-pick-place-plans
               cram-object-knowledge

               cram-cloud-logger

               cram-physics-utils     ; for reading "package://" paths
               cl-bullet ; for handling BOUNDING-BOX datastructures
               cram-bullet-reasoning
               cram-bullet-reasoning-belief-state
               cram-bullet-reasoning-utilities

               cram-location-costmap
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap
               cram-robot-pose-gaussian-costmap
               cram-occupancy-grid-costmap

               cram-urdf-projection      ; for with-simulated-robot
               cram-urdf-projection-reasoning ; to set projection reasoning to T
               cram-fetch-deliver-plans
               cram-urdf-environment-manipulation

               cram-pr2-description
               cram-boxy-description
               cram-donbot-description
               cram-hsrb-description

               cram-robokudo
               cram-giskard

               nav_msgs-msg
               manipulation_msgs-msg
	       navigation_msgs-msg
               move_base_msgs-msg
               )
  
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "knowledge-utilities" :depends-on ("package"))
             (:file "navigation-utilities" :depends-on ("package"))
             (:file "utilities" :depends-on ("package" "knowledge-utilities" "navigation-utilities"))
             (:file "process-modules" :depends-on ("package"))
             (:file "setup" :depends-on ("package"))
             (:file "cleanup-demo" :depends-on ("package"))
             (:file "storing-groceries-demo" :depends-on ("package"))
             (:file "set-the-table-demo" :depends-on ("package"))
             (:file "take-pose-client" :depends-on ("package"))))))
