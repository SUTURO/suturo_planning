(in-package :fetch)

(defparameter *tf-listener* nil)
(defparameter *planning-node* nil)

;;Init all interface clients and start a ros node
(defun init-interface()
  "Init all interfaces from planning to other groups"
  (roslisp:ros-info (init-interface) "Initialising Interfaces:")

  ;;starts ros node
  (get-planning-node)

  (init-navigation)

  (init-manipulation)

  (init-perception)

  (init-knowledge)
  (init-tts)
  (init-poi)
  (init-nlg)
  )

(defun get-planning-node ()
  (or *make-nav-plan-client*
      (init-planning)))

(defun init-navigation()
 "Initialize only local nodes for working without the real robot."

  ;;Init action clients
  (roslisp:ros-info (init-interface) "init navigation action client")
  (llif::init-nav-client))

(defun init-knowledge()

  ;;Init action clients
  (roslisp:ros-info (init-interface) "init knowledge client")
  (llif::init-knowledge-action-client)

  (llif::knowledge-set-tables-source)
  (llif::knowledge-set-ground-source)
  (llif::knowledge-set-buckets-target))

(defun init-manipulation()
  "Initialize only local nodes for working without the real robot."
  
  ;;Init action clients
  (roslisp:ros-info (init-interface) "init move grippper action client")
  (llif::init-move-gripper-action-client)

  (roslisp:ros-info (init-interface) "init grasp action client")
  (llif::init-grasp-action-client)

  (roslisp:ros-info (init-interface) "init place action client")
  (llif::init-place-action-client)
  
  (roslisp:ros-info (init-interface) "init navigation action client")
  (llif::init-nav-client)

  (roslisp:ros-info (init-interface) "init take pose action client")
  (llif:init-take-pose-action-client))

(defun init-perception()
 "Initialize only local nodes for working without the real robot."

  ;;Init client
  (roslisp:ros-info (init-interface) "init robosherlock object action client")
  (llif::init-robosherlock-object-action-client)

  ;;(roslisp:ros-info (init-clients) "init robosherlock plane action client")
  ;;(llif::init-robosherlock-plane-action-client))

(defun init-tts()
  ;;init action client
  (llif::init-text-to-speech-action-client))

(defun init-poi()
  ;;init action client
  (llif::point-listener)
  (llif::obstacle-map-listener))

(defun init-nlg()
"Initialize nlg"
  (llif::init-nlg-action-client))

(defun init-planning()
  "Initialize only local nodes for working without the real robot."  
  ;;start rosnode named planning_node
  (roslisp:ros-info (init-interface) "Creating ROS Node 'planning_node'")
  (setf *planning-node* (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)))

