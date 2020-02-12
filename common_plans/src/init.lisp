(in-package :comp)

(defun init-interface()
  "Init all interfaces from planning to other groups"
  (roslisp:ros-info (init-inteface) "Initialising Interfaces:")
  
  (init-planning)

  (init-navigation)

  (init-manipulation)

  (init-perception)
)

(defun init-navigation()
 "Initialize only local nodes for working without the real robot."

  ;;Init action clients
  (roslisp:ros-info (init-interface) "init navigation action client")
  (llif::init-nav-client)
)

(defun init-manipulation()
  "Initialize only local nodes for working without the real robot."
  
  ;;Init action clients
  (roslisp:ros-info (init-interface) "init move grippper action client")
  (llif::init-move-gripper-action-client)
 
  (roslisp:ros-info (init-interface) "init grasp action client")
  (llif::init-grasp-action-client)

  (roslisp:ros-info (init-interface) "init place action client")
  (llif::init-place-action-client)
  
  ;;Init action clients
  (roslisp:ros-info (init-interface) "init navigation action client")
  (llif::init-nav-client)
)

(defun init-perception()
 "Initialize only local nodes for working without the real robot."
  
  (roslisp:ros-info (init-interface) "init robosherlock object action client")
  (llif::init-robosherlock-object-action-client)

  ;;(roslisp:ros-info (init-clients) "init robosherlock plane action client")
  ;;(llif::init-robosherlock-plane-action-client)
)

(defun init-planning()
 "Initialize only local nodes for working without the real robot."

  (roslisp:ros-info (init-interface) "Creating ROS Node 'planning_node'")
  (roslisp-utilities:startup-ros :name "planning_node" :anonymous nil)
)
