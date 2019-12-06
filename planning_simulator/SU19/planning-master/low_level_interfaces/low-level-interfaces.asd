(defsystem low-level-interfaces
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
               cram-json-prolog)
  
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "navigation-client" :depends-on ("package"))
             (:file "giskard-poses" :depends-on ("package"))
             (:file "test-giskard" :depends-on ("package"))
             (:file "json-prolog-client" :depends-on ("package"))
             (:file "giskard-joints" :depends-on ("package"))
             (:file "robosherlock-client" :depends-on ("package"))
             (:file "marker-publisher" :depends-on ("package"))
             (:file "head-client" :depends-on ("package"))
             (:file "torso-client" :depends-on ("package"))
             (:file "joint-state-client" :depends-on ("package"))
             (:file "text-to-speech" :depends-on ("package"))
     		 (:file "vizbox-commands" :depends-on ("package"))))))

