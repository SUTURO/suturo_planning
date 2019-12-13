(defsystem low-level-interfacing
  :depends-on (roslisp
               roslisp-utilities
               cram-language
               cram-common-failures
               cl-tf
               actionlib
               geometry_msgs-msg
               tmc_msgs-msg
               std_msgs-msg
               suturo_manipulation_msgs-msg
               move_base_msgs-msg
               actionlib_msgs-msg
               control_msgs-msg
               cram-tf
               cram-simple-actionlib-client
               trajectory_msgs-msg
               controller_manager_msgs-msg
               sensor_msgs-msg
               suturo_perception_msgs-msg
               visualization_msgs-msg
               cram-json-prolog
               )
  :components
  ((:module "src"
            :components
            ((:file "package")
            (:file "manipulation-client" :depends-on ("package"))
            (:file "navigation-action" :depends-on ("package"))
	    (:file "perception-trigger-client" :depends-on ("package")) ))))
