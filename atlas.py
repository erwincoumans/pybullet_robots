import pybullet as p
import time,math
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
atlas = p.loadURDF("atlas/atlas_v4_with_multisense.urdf", [-2,3,-0.5])
for i in range (p.getNumJoints(atlas)):
	p.setJointMotorControl2(atlas,i,p.POSITION_CONTROL,0)
	print(p.getJointInfo(atlas,i))

if 1:
	objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
	zero=[0,0,0]
	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
	print("converting y to z axis")
	for o in objs:
		pos,orn = p.getBasePositionAndOrientation(o)
		y2x = p.getQuaternionFromEuler([3.14/2.,0,3.14/2])
		newpos,neworn = p.multiplyTransforms(zero,y2x,pos,orn)
		p.resetBasePositionAndOrientation(o,newpos,neworn)
else:
	p.loadURDF("plane.urdf",[0,0,-3])
	
p.loadURDF("boston_box.urdf",[-2,3,-2], useFixedBase=True)

p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=148, cameraPitch=-9, cameraTargetPosition=[0.36,5.3,-0.62])

p.loadURDF("boston_box.urdf",[0,3,-2],useFixedBase=True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

p.getCameraImage(320,200)#, renderer=p.ER_BULLET_HARDWARE_OPENGL )


t=0
p.setRealTimeSimulation(1)
while (1):
	p.setGravity(0,0,-10)
	time.sleep(0.01)
	t+=0.01
	keys = p.getKeyboardEvents()
	for k in keys:
		if (keys[k]&p.KEY_WAS_TRIGGERED):
			if (k == ord('i')):
				x = 10.*math.sin(t)
				y = 10.*math.cos(t)
				p.getCameraImage(320,200,lightDirection=[x,y,10],shadow=1)#, renderer=p.ER_BULLET_HARDWARE_OPENGL )
		
	