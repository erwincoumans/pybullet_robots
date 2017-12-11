import pybullet as p
import time
import math
from datetime import datetime

restPose = [0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
ikSolver = 0
trailDuration = 15
#lower limits for null space
ll=[-.967,-2	,-2.96,0.19,-2.96,-2.09,-3.05]
#upper limits for null space
ul=[.967,2	,2.96,2.29,2.96,2.09,3.05]
#joint ranges for null space
jr=[5.8,4,5.8,4,5.8,4,6]
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
THRESHOLD = .5
LOWER_LIMITS = [-.967, -2.0, -2.96, 0.19, -2.96, -2.09, -3.05]
UPPER_LIMITS = [.96, 2.0, 2.96, 2.29, 2.96, 2.09, 3.05]
JOINT_RANGE = [5.8, 4, 5.8, 4, 5.8, 4, 6]
REST_POSE = [0, 0, 0, math.pi / 2, 0, -math.pi * 0.66, 0]
JOINT_DAMP = [0.1]*10
REST_JOINT_POS = [-0., -0., 0., 1.570793, 0., -1.036725, 0.000001]
MAX_FORCE = 500

def euc_dist(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist
	
class KukaDemo():
	def __init__(self):
		
		objects = [p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000,-0.200000,0.600000,0.000000,0.000000,0.000000,1.000000)]
		kuka = objects[0]
		self._kukaId = kuka
		self.kuka = kuka
		
		jointPositions=[ -0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001 ]
		for jointIndex in range (p.getNumJoints(kuka)):
			p.resetJointState(kuka,jointIndex,jointPositions[jointIndex])
			p.setJointMotorControl2(kuka,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

		p.resetBasePositionAndOrientation(self._kukaId,[5.2,2.6,-1.3],[0,0,0,1])
		p.stepSimulation()
		
		objects = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")
		kuka_gripper = objects[0]
		print ("kuka gripper=")
		print(kuka_gripper)
		self.kuka_gripper = kuka_gripper

		kuka_cid = p.createConstraint(kuka,   6,  kuka_gripper,0,p.JOINT_FIXED, [0,0,0], [0,0,0.05],[0,0,0])
		self.kuka_cid = kuka_cid
		pr2_cid2 = p.createConstraint(kuka_gripper,4,kuka_gripper,6,jointType=p.JOINT_GEAR,jointAxis =[1,1,1],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
		p.changeConstraint(pr2_cid2,gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)
		self.pr2_cid2 = pr2_cid2		
		self.objectsA = p.loadURDF("sphere_small.urdf", 4.692311,2.612670,-1.326377,0.537650,0.466365,-0.527710,0.463636)
		self.objectsB = p.loadURDF("cube_small.urdf", 4.728484,2.478686,-1.327714,0.383040,0.597531,-0.372735,0.597751)
		self.objectsC = p.loadURDF("cube_small.urdf", 4.688386,2.394517,-1.329060,0.518556,0.487225,-0.508511,0.484899)
		self.objectsD = p.loadURDF("cube_small.urdf", 4.631109,2.505806,-1.328169,0.348580,0.617677,-0.338303,0.618480)
		self.objectsE = p.loadURDF("duck_vhacd.urdf", 4.848411,2.450100,-1.327096,0.471705,0.531978,-0.461491,0.530585)
		self.objectsF = p.loadURDF("cube_small.urdf", 4.707141,2.276235,-1.267120,0.430410,0.523182,-0.387767,0.625032)
		
		self.objects0 = p.loadURDF("jenga/jenga.urdf", 4.696415,2.771864,-1.274404,-0.217063,0.667856,0.223462,0.675956)
		self.objects1 = p.loadURDF("jenga/jenga.urdf", 4.752309,2.797695,-1.273640,-0.124988,0.691698,0.132404,0.698858)
		self.objects2 = p.loadURDF("jenga/jenga.urdf", 4.817710,2.810722,-1.272955,-0.061221,0.700847,0.069246,0.707298)
		self.objects3 = p.loadURDF("jenga/jenga.urdf", 4.884461,2.810587,-1.272419,-0.104396,0.695290,0.112014,0.702230)
		self.objects4 = p.loadURDF("jenga/jenga.urdf", 4.955992,2.815171,-1.271788,-0.076916,0.699130,0.084800,0.705769)
		self.objects5 = p.loadURDF("jenga/jenga.urdf", 5.023587,2.816969,-1.271221,-0.122495,0.692164,0.129933,0.699301)
		self.objects6 = p.loadURDF("jenga/jenga.urdf", 5.085391,2.813410,-1.228314,-0.189990,0.630994,0.114184,0.743446)
		self.reset()
		
	def reset(self):
		p.resetBasePositionAndOrientation(self.objectsA,[4.692311,2.612670,-1.326377],[0.537650,0.466365,-0.527710,0.463636])
		p.resetBasePositionAndOrientation(self.objectsB,[4.728484,2.478686,-1.327714],[0.383040,0.597531,-0.372735,0.597751])
		p.resetBasePositionAndOrientation(self.objectsC,[4.688386,2.394517,-1.329060],[0.518556,0.487225,-0.508511,0.484899])
		p.resetBasePositionAndOrientation(self.objectsD,[4.631109,2.505806,-1.328169],[0.348580,0.617677,-0.338303,0.618480])
		p.resetBasePositionAndOrientation(self.objectsE,[4.848411,2.450100,-1.327096],[0.471705,0.531978,-0.461491,0.530585])
		p.resetBasePositionAndOrientation(self.objectsF,[4.707141,2.276235,-1.267120],[0.430410,0.523182,-0.387767,0.625032])

		p.resetBasePositionAndOrientation(self.objects0,[4.696415,2.771864,-1.274404],[-0.217063,0.667856,0.223462,0.675956])
		p.resetBasePositionAndOrientation(self.objects1,[4.752309,2.797695,-1.273640],[-0.124988,0.691698,0.132404,0.698858])
		p.resetBasePositionAndOrientation(self.objects2,[4.817710,2.810722,-1.272955],[-0.061221,0.700847,0.069246,0.707298])
		p.resetBasePositionAndOrientation(self.objects3,[4.884461,2.810587,-1.272419],[-0.104396,0.695290,0.112014,0.702230])
		p.resetBasePositionAndOrientation(self.objects4,[4.955992,2.815171,-1.271788],[-0.076916,0.699130,0.084800,0.705769])
		p.resetBasePositionAndOrientation(self.objects5,[5.023587,2.816969,-1.271221],[-0.122495,0.692164,0.129933,0.699301])
		p.resetBasePositionAndOrientation(self.objects6,[5.085391,2.813410,-1.228314],[-0.189990,0.630994,0.114184,0.743446])

		
		p.resetBasePositionAndOrientation(self.kuka_gripper,[0.923103,-0.200000,1.250036],[-0.000000,0.964531,-0.000002,-0.263970])
		jointPositions=[ 0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000 ]
		for jointIndex in range (p.getNumJoints(self.kuka_gripper)):
			p.resetJointState(self.kuka_gripper,jointIndex,jointPositions[jointIndex])
			p.setJointMotorControl2(self.kuka_gripper,jointIndex,p.POSITION_CONTROL,jointPositions[jointIndex],0)

		global restPose
		for jointIndex in range(6):
				p.setJointMotorControl2(self._kukaId, jointIndex, p.POSITION_CONTROL, 
					restPose[jointIndex], 0)
				p.resetJointState(self._kukaId, jointIndex,restPose[jointIndex])
		print ("kuka reset")

		
	def update(self, context):
		dt = datetime.now()
		self._t = (dt.second/60.)*2.*math.pi
		global REST_JOINT_POS
		if (context.validPos):
			pos = context.pos
			orn = context.orn
			trigger_press = context.analog
		
			i=4
			p.setJointMotorControl2(self.kuka_gripper, i, p.POSITION_CONTROL, targetPosition=trigger_press*0.05, force=10)
			i=6
			p.setJointMotorControl2(self.kuka_gripper, i, p.POSITION_CONTROL, targetPosition=trigger_press*0.05, force=10)
			sq_len = euc_dist(p.getLinkState(self.kuka, 6)[0],pos)

			if sq_len < THRESHOLD * THRESHOLD:
				eef_pos = pos
				eef_orn = p.getQuaternionFromEuler([0,-math.pi,0])#orn = context.orn
				joint_pos = p.calculateInverseKinematics(self.kuka, 6, eef_pos, eef_orn,
					lowerLimits=LOWER_LIMITS, upperLimits=UPPER_LIMITS, 
					jointRanges=JOINT_RANGE, restPoses=REST_POSE, jointDamping=JOINT_DAMP)

				
				for i in range(len(joint_pos)):
					p.setJointMotorControl2(self.kuka, i, p.POSITION_CONTROL, 
						targetPosition=joint_pos[i], targetVelocity=0, positionGain=0.15, 
						velocityGain=1.0, force=MAX_FORCE)


			else:
				# Set back to original rest pose
				for jointIndex in range(p.getNumJoints(self.kuka)):
					p.setJointMotorControl2(self.kuka, jointIndex, p.POSITION_CONTROL, 
						REST_JOINT_POS[jointIndex], 0)
		#jointPoses = p.calculateInverseKinematics(self._kukaId,self._kukaEndEffectorIndex,pos,orn,jointDamping=self._jointDamping,solver=ikSolver)
		#jointPoses = p.calculateInverseKinematics(self._kukaId,self._kukaEndEffectorIndex,pos,orn,jointDamping=self._jointDamping,solver=ikSolver)
		#jointPoses = p.calculateInverseKinematics(self._kukaId,self._kukaEndEffectorIndex,pos,orn,lowerLimits=ll,upperLimits=ul,jointRanges=jr,restPoses=restPose,jointDamping=self._jointDamping,solver=ikSolver)
		
		