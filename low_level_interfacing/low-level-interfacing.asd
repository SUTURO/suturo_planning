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
               std_srvs-srv
               manipulation_msgs-msg
               knowledge_msgs-msg
               nlg_msgs-msg
               cram-prolog
               cram-designators
               cram-process-modules
               cram-language-designator-support
               cram-executive)
  :components
  ((:module "src"
      :components
      ((:file "package")
      (:file "move-gripper-client" :depends-on ("package"))
      (:file "navigation-action" :depends-on ("package"))
      (:file "grasp-action-client" :depends-on ("package")) 
      (:file "obstacle-map-subscriber" :depends-on ("package")) 
      (:file "place-action-client" :depends-on ("package"))
      (:file "open-door-client" :depends-on ("package"))
	    (:file "marker-publisher" :depends-on ("package"))
	    (:file "robosherlock-client-object" :depends-on ("package"))
	    (:file "robosherlock-client-plane" :depends-on ("package"))
	    (:file "robosherlock-client-door" :depends-on ("package"))
      (:file "knowledge-insertion-client" :depends-on ("package"))
	    (:file "knowledge-client" :depends-on ("package"))
      (:file "take-pose-action-client" :depends-on ("package"))
	    (:file "nlp-subscriber" :depends-on ("package"))
	    (:file "poi-client" :depends-on("package"))
	    (:file "text-to-speech" :depends-on("package"))
      (:file "nlg-action-client" :depends-on("package"))
      (:file "make-plan-action-client" :depends-on("package"))))))
