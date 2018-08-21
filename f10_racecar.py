import pybullet as p
import time
import math

p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0,0,-10)
useRealTimeSim = 1

p.setRealTimeSimulation(useRealTimeSim) # either this

track = p.loadSDF("f10_racecar/meshes/barca_track.sdf", globalScaling=1)
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0,0,1])


	
for wheel in range(p.getNumJoints(car)):
	p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
	p.getJointInfo(car,wheel)	

wheels = [8,15]
print("----------------")

#p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car,9,car,11,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=1, maxForce=10000)

c = p.createConstraint(car,10,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,9,car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,16,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=1, maxForce=10000)


c = p.createConstraint(car,16,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,17,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, maxForce=10000)

c = p.createConstraint(car,1,car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15, maxForce=10000)
c = p.createConstraint(car,3,car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15,maxForce=10000)


steering = [0,2]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
steeringSlider = p.addUserDebugParameter("steering",-1,1,0)

def getCarYaw(car):
	carPos,carOrn = p.getBasePositionAndOrientation(car)
	carEuler = p.getEulerFromQuaternion(carOrn)
        carYaw = carEuler[2]*360/(2.*math.pi)-90
	return carYaw

prevCarYaw = getCarYaw(car)

while (True):
	carPos,carOrn = p.getBasePositionAndOrientation(car)

	# Keep the previous orientation of the camera set by the user.
	camInfo = p.getDebugVisualizerCamera()
	yaw = camInfo[8]
	pitch = camInfo[9]
	distance = camInfo[10]
	targetPos = camInfo[11]
	camFwd = camInfo[5]

	carYaw = getCarYaw(car)

	#the car yaw is clamped between -90 and 270, make sure to deal with angles that wrap around
	if (carYaw-prevCarYaw>45):
		yaw+=360
	if (carYaw-prevCarYaw<-45):
		yaw-=360
	prevCarYaw = carYaw

	#print("carYaw=", carYaw)
	#print("camYaw=", yaw)
	
	#slowly rotate the camera behind the car
	diffYaw = (carYaw-yaw)*0.03

	#track the position of the car as target
	p.resetDebugVisualizerCamera(distance, yaw+diffYaw, pitch, carPos)

	maxForce = p.readUserDebugParameter(maxForceSlider)
	targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
	steeringAngle = p.readUserDebugParameter(steeringSlider)
	#print(targetVelocity)
	
	for wheel in wheels:
		p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
		
	for steer in steering:
		p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=-steeringAngle)
		
	steering
	if (useRealTimeSim==0):
		p.stepSimulation()
	time.sleep(1./240.)
