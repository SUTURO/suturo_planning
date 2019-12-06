(in-package :plc)

(defun init-planning ()
  "Initialize all the interfaces from planning to other groups."
  (print "init ros node..")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil);;)
  (cram-tf::init-tf)
  
  ;;Init all the action clients
  (roslisp:ros-info (init-clients) "init tf listener")
  (plc::get-tf-listener)
  
  (roslisp:ros-info (init-clients) "init text to speech client")
  (lli:init-text-to-speech-action-client) ;; for text-to-speech

  (roslisp:ros-info (init-clients) "init navigation client")
  (lli:init-nav-client)

  (roslisp:ros-info (init-clients) "init move head action  client")
  (lli:init-move-head-action-client)

  (roslisp:ros-info (init-clients) "init giskard joints action client")
  (lli:init-giskard-joints-action-client)

  (roslisp:ros-info (init-clients) "init robosherlock action client")
  (lli:init-robosherlock-action-client)

  ;;(roslisp:ros-info (init-clients) "init vizbox publisher")
  ;;(lli:viz-box-init)

  (roslisp:ros-info (init-clients) "init visualization marker publisher")
  (lli::init-marker-publisher)
  ;;(lli:init-move-torso-action-client) ;;NOTE works via giskard now! :D
   ;;(lli:make-giskard-poses-action-client)

  (roslisp:ros-info (init-clients) "All action clients are set up.")) ;; for navigation
  
(defun init-integration()
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (roslisp:ros-info (init-clients) "init speech client")
  (lli:init-text-to-speech-action-client) ;; for text-to-speech
  (roslisp:ros-info (init-clients) "init navigation client")
  (lli:init-nav-client)
  (roslisp:ros-info (init-clients) "init moce-head client")
  (lli:init-move-head-action-client)
  (roslisp:ros-info (init-clients) "init giskard-joints")
  (lli:init-giskard-joints-action-client)
  (roslisp:ros-info (init-clients) "init move-torso client")
  (lli:init-move-torso-action-client)
  (roslisp:ros-info (init-clients) "init tf-listener")
  (plc:get-tf-listener)
  (roslisp:ros-info (init-clients) "init viz-box")
  (lli:viz-box-init))


(defun init-local()
  "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (print "init tf-listener")
  (plc::get-tf-listener)
  (print "init viz-box")
  (lli:viz-box-init)
  (print "init marker publisher")
  (lli:init-marker-publisher))
