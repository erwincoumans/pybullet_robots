import pybullet as p
import time
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
atlas = p.loadURDF("atlas/atlas_v4_with_multisense.urdf")
for i in range (p.getNumJoints(atlas)):
	p.setJointMotorControl2(atlas,i,p.POSITION_CONTROL,0)

p.loadURDF("plane.urdf",[0,0,-1])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

p.setRealTimeSimulation(1)
p.stepSimulation()
p.getCameraImage(320,200)
while (1):
	p.setGravity(0,0,-10)
	time.sleep(0.01)
	#p.getCameraImage(320,200)
	