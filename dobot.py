#Created and copyright by Erwin Coumans for http://pybullet.org
import pybullet as p
p.connect(p.GUI)
#p.loadURDF("plane_transparent2.urdf")
p.loadURDF("plane.urdf")
dobot = p.loadURDF("dobot/dobot.urdf",useFixedBase=True)
p.setRealTimeSimulation(1)
p.setPhysicsEngineParameter(numSolverIterations=300)
p.setPhysicsEngineParameter(numSubSteps=10)

for i in range (p.getNumJoints(dobot)):
	print(p.getJointInfo(dobot,i))
	p.setJointMotorControl2(dobot,i,p.VELOCITY_CONTROL,targetVelocity=0,force=20)
	
#p.setJointMotorControl2(dobot,1,p.VELOCITY_CONTROL,targetVelocity=0.1,force=100)
#p.setJointMotorControl2(dobot,2,p.VELOCITY_CONTROL,targetVelocity=0.,force=100)

p.setJointMotorControl2(dobot,0,p.VELOCITY_CONTROL,targetVelocity=0,force=1000)
p.setJointMotorControl2(dobot,3,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(dobot,5,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(dobot,6,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(dobot,8,p.VELOCITY_CONTROL,targetVelocity=0,force=0)

p.resetJointState(dobot,6,-1.57)
p.resetJointState(dobot,7,-1.57)
p.resetJointState(dobot,8,1.57)

#colors for debugging
#p.changeVisualShape(dobot,4,rgbaColor=[1,1,1,1])
#p.changeVisualShape(dobot,3,rgbaColor=[0,1,0,1])
#p.changeVisualShape(dobot,6,rgbaColor=[0,1,0,1])
#p.changeVisualShape(dobot,5,rgbaColor=[1,1,0,1])

c = p.createConstraint(dobot,6,dobot,8,jointType=p.JOINT_POINT2POINT,jointAxis =[1,0,0],parentFramePosition=[-0.05,0,0.08],childFramePosition=[-0.05,0.026,-0.006])#0.014-0.02 (inertial com)

c = p.createConstraint(dobot,3,dobot,5,jointType=p.JOINT_POINT2POINT,jointAxis =[1,0,0],parentFramePosition=[-0.05,0,0.075],childFramePosition=[-0.05,-0.026,0.014])

0.150
p.getCameraImage(320,200)#160,100)
p.resetDebugVisualizerCamera(1,85.6,0,[-0.61,0.12,0.25])
p.setJointMotorControl2(dobot,6,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
for i in range (p.getNumJoints(dobot)):
	worldLinkFramePosition = p.getLinkState(dobot,i)[4]
	worldLinkFrameOrientation = p.getLinkState(dobot,i)[5]
	print(i,worldLinkFramePosition)
	
while (1):
	p.setGravity(0,0,0)
	ls = p.getLinkState(dobot,5)
	linkOrn = ls[1]
	eul = p.getEulerFromQuaternion(linkOrn)
	#print(eul)
	
	
	
	