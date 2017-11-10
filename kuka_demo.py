import pybullet as p
import time
import math
from datetime import datetime

restPose = [0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
ikSolver = 0
trailDuration = 15


class KukaDemo():
	def __init__(self):
		kukaId = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
		self._kukaId = kukaId[0]
		p.resetBasePositionAndOrientation(self._kukaId,[4.8,3.5,-1.2],[0,0,0,1])
		self._kukaEndEffectorIndex = 6
		self._numJoints = p.getNumJoints(self._kukaId)
		self._jointDamping = [0.1]*self._numJoints
		print('numJoints')
		print(self._numJoints)
		self._t = 0
		self._hasPrevPose = 0
		self._prevPose=[0,0,0]
		self._prevPose1=[0,0,0]
		self._gripperMaxAngle = 1.0
		self._fingerJointIndices = (8, 10, 11, 13)
		self._maxTorqueValues = (1.0, 0.5, 1.0, 0.5)
		self._fingerJointsOpen = (0, 0.1, 0, -0.1)
		self._fingerJointsClose = (self._gripperMaxAngle, 0.1,
									self._gripperMaxAngle, -0.1)
		self._controlMode = p.POSITION_CONTROL
		
	def reset(self):
		print ("kuka reset")
	
	def controlGripperAngle(self, finger_angle):
		fingerTargetPositions = [self._fingerJointsClose[0] - finger_angle, self._fingerJointsClose[1], self._fingerJointsClose[2] - finger_angle, self._fingerJointsClose[3]]
		self.setGripperJointAngles(fingerTargetPositions)

	def setGripperJointAngles(self, target_angles):
		p.setJointMotorControlArray(self._kukaId,self.fingerJointIndices,controlMode=self._controlMode,targetPositions=target_angles,targetVelocities=[0.0] * len(target_angles),forces=self._maxTorqueValues,positionGains=[0.2] * len(target_angles),velocityGains=[1.0] * len(target_angles))

	def closeGripper(self, num_steps=1, duration=0.0, step_simulation=False):
		for i in range(num_steps):
			fingerAngle = self._gripperMaxAngle * (num_steps - i - 1) / float(num_steps)
			self.controlGripperAngle(finger_angle)
			time.sleep(duration / float(num_steps))

	def openGripper(self, num_steps=1, duration=0.0):
		for i in range(num_steps):
			fingerAngle = self._gripperMaxAngle * (i + 1.0) / float(num_steps)
			self.controlGripperAngle(finger_angle)
			time.sleep(duration / float(num_steps))
		
	def update(self):
		dt = datetime.now()
		self._t = (dt.second/60.)*2.*math.pi

		pos = [4.8-0.4,3.5+0.2*math.cos(self._t),-1.2+0.2*math.sin(self._t)]
		orn = p.getQuaternionFromEuler([0,-math.pi,0])
		
		jointPoses = p.calculateInverseKinematics(self._kukaId,self._kukaEndEffectorIndex,pos,orn,jointDamping=self._jointDamping,solver=ikSolver)
		
		for i in range(self._numJoints):
			jointInfo = p.getJointInfo(self._kukaId, i)
			qIndex = jointInfo[3]
			if qIndex > -1:
				p.setJointMotorControl2(bodyIndex=self._kukaId,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[qIndex-7],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
		ls = p.getLinkState(self._kukaId,self._kukaEndEffectorIndex)
		if (self._hasPrevPose):
			p.addUserDebugLine(self._prevPose,pos,[0,0,0.3],1,trailDuration)
			p.addUserDebugLine(self._prevPose1,ls[4],[1,0,0],1,trailDuration)
		self._prevPose=pos
		self._prevPose1=ls[4]
		self._hasPrevPose = 1
