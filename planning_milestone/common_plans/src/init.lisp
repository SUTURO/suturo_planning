(in-package :comp)

(defun init-interface()
  "Init all interfaces from planning to other groups"
  (print "init ros node...")

  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (cram-tf::init-tf)

  ;;Init action clients
  (roslisp:ros-info (init-clients) "init navigation action client")
  (llif::init-nav-client)

  (roslisp:ros-info (init-clients) "init move grippper action client")
  (llif::init-move-gripper-action-client)
 
  (roslisp:ros-info (init-clients) "init grasp action client")
  (llif::init-grasp-action-client)

  (roslisp:ros-info (init-clients) "init place action client")
  (llif::init-place-action-client)
  
  (roslisp:ros-info (init-clients) "init perceive action client ")
  (llif::init-perceive-action-client)

  (roslisp:ros-info (init-clients) "init robosherlock action client")
  (llif::init-robosherlock-action-client)
)

(defun init-navigation()
 "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (print "init tf-listener")
  (comp::get-tf-listener)

  ;;Init action clients
  (roslisp:ros-info (init-clients) "init navigation action client")
  (llif::init-nav-client)
)

(defun init-manipulation()
 "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (print "init tf-listener")
  (comp::get-tf-listener)

  ;;Init action clients
  (roslisp:ros-info (init-clients) "init move grippper action client")
  (llif::init-move-gripper-action-client)
 
  (roslisp:ros-info (init-clients) "init grasp action client")
  (llif::init-grasp-action-client)

  (roslisp:ros-info (init-clients) "init place action client")
  (llif::init-place-action-client)
  
  (roslisp:ros-info (init-clients) "init perceive action client ")
  (llif::init-perceive-action-client)

  ;;Init action clients
  (roslisp:ros-info (init-clients) "init navigation action client")
  (llif::init-nav-client)
)

(defun init-perception()
 "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (print "init tf-listener")
  (comp::get-tf-listener)
 
  ;;Init action clients
  (roslisp:ros-info (init-clients) "init robosherlock action client")
  (llif::init-robosherlock-action-client)
	
)

(defun init-planning()
 "Initialize only local nodes for working without the real robot."
  (print "create ros node")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
  (print "init tf-listener")
  (comp::get-tf-listener)
)
