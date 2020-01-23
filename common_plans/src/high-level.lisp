(in-package :comp)


(defun execute-m1()
  (init-manipulation)
	;;call perceive takes torso_height and pose 
	(llif::call-perceive-action 0 2)
	;;call nav action takes x y z as goal position
	(llif::call-nav-action 0.2 1 3) 
	(llif::listener)
	(llif::trigger-perception-pipeline-main)
	;;(llif::call-place-action -0.87 0.824 0.28 0 0.7 0 0.7)
)
