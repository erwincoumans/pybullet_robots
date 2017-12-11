import pybullet as p
p.connect(p.GUI)
#p.loadURDF("plane_transparent2.urdf")
p.loadURDF("plane.urdf")
dobot = p.loadURDF("dobot/dobot.urdf",[0,0,0.1], useFixedBase=True)
p.setRealTimeSimulation(1)
for i in range (p.getNumJoints(dobot)):
	print(p.getJointInfo(dobot,i))
	p.setJointMotorControl2(dobot,i,p.VELOCITY_CONTROL,targetVelocity=0,force=20)
	
#p.setJointMotorControl2(dobot,1,p.VELOCITY_CONTROL,targetVelocity=0.1,force=100)
#p.setJointMotorControl2(dobot,2,p.VELOCITY_CONTROL,targetVelocity=0.,force=100)

p.setJointMotorControl2(dobot,0,p.VELOCITY_CONTROL,targetVelocity=0,force=1000)
p.setJointMotorControl2(dobot,3,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(dobot,5,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
p.setJointMotorControl2(dobot,6,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
#p.resetJointState(dobot,4,1.57)
p.resetJointState(dobot,7,-1.57)
p.resetJointState(dobot,8,2.57)


c = p.createConstraint(dobot,6,dobot,8,jointType=p.JOINT_POINT2POINT,jointAxis =[1,0,0],parentFramePosition=[-0.05,0,0.075],childFramePosition=[-0.05,0.025,-0.0242])

c = p.createConstraint(dobot,3,dobot,5,jointType=p.JOINT_POINT2POINT,jointAxis =[1,0,0],parentFramePosition=[-0.05,0,0.085],childFramePosition=[-0.05,-0.026,0.014])


while (1):
	p.setGravity(0,0,0)
	p.getCameraImage(320,200)#160,100)
