(in-package :clean)

(defparameter *tf-listener* nil)
(defparameter *planning-node* nil)

;;Init all interface clients and start a ros node
(defun init-interfaces()
  "Init all interfaces from planning to other groups"
  (setq inferior-lisp-program "sbcl --dynamic-space-size 1024")
  (roslisp:ros-info (init-interfaces) "Initialising Interfaces:")


  ;;starts ros node
  (get-planning-node) 
  
  (init-planning)

  (init-navigation)

  (init-manipulation)

  (init-perception)

  (init-nlg)
  (init-knowledge)

  (init-tts)

  (init-nlg)

  (init-poi))

(defun get-planning-node ()
  (or *planning-node*
      (init-planning)))

(defun init-navigation()
 "Initialize only local nodes for working without the real robot."
  ;;Init action clients
  (roslisp:ros-info (init-interfaces) "init navigation action client")
  (llif::init-nav-client))

(defun init-knowledge()
  ;;Init action clients
  (roslisp:ros-info (init-interfaces) "init knowledge client")
  (llif::init-knowledge-action-client)

  (llif::knowledge-set-tables-source)
  (llif::knowledge-set-ground-source)
  (llif::knowledge-set-buckets-target))

(defun init-manipulation()
  "Initialize only local nodes for working without the real robot."
  
  ;;Init action clients
  (roslisp:ros-info (init-interfaces) "init move grippper action client")
  (llif::init-move-gripper-action-client)

  (roslisp:ros-info (init-interfaces) "init grasp action client")
  (llif::init-grasp-action-client)

  (roslisp:ros-info (init-interfaces) "init place action client")
  (llif::init-place-action-client)
  
  (roslisp:ros-info (init-interfaces) "init navigation action client")
  (llif::init-nav-client)

  (roslisp:ros-info (init-interfaces) "init take pose action client")
  (llif:init-take-pose-action-client)

  (roslisp:ros-info (init-interfaces) "init make plan action client")
  (llif::init-make-plan-action-client))

(defun init-perception()
 "Initialize only local nodes for working without the real robot."

  ;;Init client
  (roslisp:ros-info (init-interfaces) "init robosherlock object action client")
  (llif::init-robosherlock-object-action-client))
  ;;(roslisp:ros-info (init-clients) "init robosherlock plane action client")
  ;;(llif::init-robosherlock-plane-action-client))

(defun init-tts()
  ;;init action client
  (llif::init-text-to-speech-action-client))

(defun init-nlg()
  (roslisp:ros-info (init-interfaces) "init nlg action client")
  (llif::init-nlg-action-client))
(defun init-poi()
  ;;init action client
  (roslisp:ros-info (init-interfaces) "init point of interest scanning")
  (llif::point-listener)
  (llif::obstacle-map-listener))


(defun init-planning()
  "Initialize only local nodes for working without the real robot."  
  ;;start rosnode named planning_node
  (roslisp:ros-info (init-interfaces) "Creating ROS Node 'planning_node'")
  (setf *planning-node* (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)))
