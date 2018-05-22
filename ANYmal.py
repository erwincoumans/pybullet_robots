import pybullet as p
import time
p.connect(p.GUI)

ground = p.loadURDF("plane.urdf",[0,0,0])
p.changeVisualShape(ground,-1,rgbaColor=[1,1,1,0.8])
p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION,1)

anymal = p.loadURDF("ANYmal/robot.urdf",[0,0,0.6])
for j in range(p.getNumJoints(anymal)):
	p.setJointMotorControl2(anymal,j,p.POSITION_CONTROL,targetPosition=0, force=10)


p.setGravity(0,0,-10)
while (1):
	p.stepSimulation()
	time.sleep(1./240.)
	
		