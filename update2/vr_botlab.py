import pybullet as p
import time
import math
import copy
import minitaur_demo
import kuka_demo
import pendulum_demo
import empty_demo
import pybullet_data
from datetime import datetime

#vr_shift=[-1,-0.6,-1]
vr_shift=[1.9000000000000004, -0.30000000000000004, -1]
					
update_all_demos = False

useMaximalCoordinatesEnvObjects=False #there is some issue with maximal coordinate bodies

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
p.resetSimulation()

sphereRadius = 0.02
meshScale = [.1,.1,.01]
shift = [0,0,0]
plugin = -1
uiCube = -1
pr2_gripper = -1
numLines = 1
lines = [-1]*numLines
pr2_cid = -1
pr2_cid2 = -1
textOrn = [0,0,0,1]

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
ANALOG=3
BUTTONS=6
frameNr = 0
objectInfo = ""
pointRay = -1
rayLen = 100
buttonA = 0
controllerId = -1
uiControllerId = -1
t7=0
currentDemo = 0
demos = []
plugin = p.loadPlugin("vrSyncPlugin")


class Context():
	def __init__(self):
		self.validPos = False
		self.vrMode = True
		self.pos = [0,0,0]
		self.analog=0
		self.allAnalog = [0,0,0,0,0,0,0,0]
		
def resetAll():
	print("p.resetSimulation()!!!!!!!!!!!!!!")
	p.resetSimulation()
	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])
	
	visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="marble_cube.obj", rgbaColor=[1,1,1,.3], specularColor=[1,1,1], visualFramePosition=shift, meshScale=meshScale)
	#collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="marble_cube.obj", collisionFramePosition=shift,meshScale=meshScale)
	collisionShapeId = -1
	
	global uiCube
	uiCube = p.createMultiBody(baseMass=0,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = [0,1,0], useMaximalCoordinates=False)
	p.changeVisualShape(uiCube,0,rgbaColor=[0,0,0,1])
	global textOrn
	textOrn = p.getQuaternionFromEuler([0,0,-1.5707963])

	p.stepSimulation()
	#disable rendering during loading makes it much faster
	p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
	p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
	p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)



	#objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
	#kitchenObj = p.loadSDF("kitchens/1.sdf")
	if 1:
		botlabobjects = p.loadSDF("botlab/botlab.sdf", globalScaling=2, useMaximalCoordinates=useMaximalCoordinatesEnvObjects)
		#botlabobjects = p.loadSDF("botlab/newsdf.sdf", globalScaling=2)
		print("num botlabobjects  = ", botlabobjects )

		for o in botlabobjects:
			pos,orn = p.getBasePositionAndOrientation(o)
			zero=[0,0,0]#[0,1,2.2]
			y2x = p.getQuaternionFromEuler([3.14/2.,0,-0])
			newpos,neworn = p.multiplyTransforms(zero,y2x,pos,orn)
			p.resetBasePositionAndOrientation(o,newpos,neworn)
	else:
		p.loadURDF("plane.urdf",[0,0,-2])
	global pr2_gripper
	#objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
	objects = [p.loadURDF("pr2_gripper2.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
	pr2_gripper = objects[0]
	print ("pr2_gripper=")
	print (pr2_gripper)

	jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
	for jointIndex in range (p.getNumJoints(pr2_gripper)):
		p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])
		p.setJointMotorControl2(pr2_gripper,jointIndex,p.POSITION_CONTROL,targetPosition=0,force=0)
	global pr2_cid
	pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])#[0.500000,0.300006,0.200000])
	print ("pr2_cid")
	print (pr2_cid)

	p.changeConstraint(pr2_cid, maxForce=0)
	global pr2_cid2
	pr2_cid2 = p.createConstraint(pr2_gripper,0,pr2_gripper,2,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
	p.changeConstraint(pr2_cid2,gearRatio=1, erp=0.5, relativePositionTarget=0.5, maxForce=3)
	p.resetBasePositionAndOrientation(pr2_gripper,[0,0,-10], [0,0,0,1])


	#r2d2 = p.loadURDF("r2d2.urdf", [0.895528,-1.177031,-1.541080],[0.000000,0.000000,0.000000,1.000000])
	#for i in range(p.getNumJoints(r2d2)):
	#	print(p.getJointInfo(r2d2,i))
	#p.setJointMotorControl2(r2d2,13,p.VELOCITY_CONTROL,targetVelocity=3)
	#for i in [2,3,6,7]:
	#	p.setJointMotorControl2(r2d2,i,p.VELOCITY_CONTROL,targetVelocity=-2)

	#objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
	#objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
	#objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
	#objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]

	
	p.setGravity(0,0,-10)

	p.setRealTimeSimulation(1)



	p.configureDebugVisualizer(p.COV_ENABLE_VR_RENDER_CONTROLLERS, 0)
	p.configureDebugVisualizer(p.COV_ENABLE_VR_PICKING, 0)
	p.configureDebugVisualizer(p.COV_ENABLE_VR_TELEPORTING, 0)


	#print("logId userdebug")
	#print(logId)
	
	for i in range (numLines):
		spacing = 0.01
		textPos = [.1-(i+1)*spacing,.1,0.011]
		text = "ABCDEFGH"*10
		lines[i] = p.addUserDebugText(text,textColorRGB=[0,0,0], textSize = 0.01, textPosition = textPos,textOrientation = textOrn,parentObjectUniqueId=uiCube, parentLinkIndex = -1)
		
	#if (once):
	#	once = 0
	#	if (logId and logId>0):
	#		p.stopStateLogging(logId)

	global kuka_demo
	global minitaur_demo
	global pendulum_demo
	minitaur_demo_instance = minitaur_demo.MinitaurDemo() #empty_demo.EmptyDemo()#
	#minitaur_demo_instance = EmptyDemo()#minitaur_demo.MinitaurDemo()
	kuka_demo_instance = kuka_demo.KukaDemo()
	pendulum_demo_instance = pendulum_demo.PendulumDemo()
	kuka_grasping_instance = pendulum_demo.KukaGraspingDemo()

					
	demos=[	
	["minitaur\nUse analog stick!",			[1.0099999904632568,-34.40010452270508,4.5999579429626465, [-2.699556350708008, -2.7035043239593506, -1.1882244348526]],	minitaur_demo_instance,[1,1./240.,50]],
	["kuka lfd",	[1.0,163.59983825683594, -17.799997329711914, [4.711545467376709, 4.462226867675781, -0.824081897735596]],												kuka_demo_instance,		 [1,1./240.,50]],
	["kuka_grasping\nEvolution Strategies\nNo interaction.",	[1.0,163.59983825683594, -17.799997329711914, [2.011545467376709, 2.92226867675781, -1.0824081897735596]],	kuka_grasping_instance,[0,1./240.,50]],
#	["pendulum",			[1.0,-448.40008544921875, -11.000036239624023, [-1.0783566236495972, 1.9088447690010071, -1.105987787246704]],		 				pendulum_demo_instance,[1,0.0165, 5]],

		]
		
	global currentDemo
	global demos
	
	p.resetDebugVisualizerCamera(demos[currentDemo][1][0],
												 demos[currentDemo][1][1],
												 demos[currentDemo][1][2],
												 demos[currentDemo][1][3])
	p.setVRCameraState(rootPosition=demos[currentDemo][1][3])

	p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

	global controllerId
	global uiControllerId
	print("waiting for VR UI controller trigger")
	while (uiControllerId<0):
		time.sleep(0.1)
		events = p.getVREvents()
		for e in (events):
			if (e[BUTTONS][33]==p.VR_BUTTON_IS_DOWN):
				uiControllerId = e[CONTROLLER_ID]
			if (e[BUTTONS][32]==p.VR_BUTTON_IS_DOWN):
				uiControllerId = e[CONTROLLER_ID]

	print("Using uiControllerId="+str(uiControllerId))

	print("waiting for VR picking controller trigger")
	while (controllerId<0):
		time.sleep(0.1)
		events = p.getVREvents()
		for e in (events):
			if (e[BUTTONS][33]==p.VR_BUTTON_IS_DOWN):
				controllerId = e[CONTROLLER_ID]
			if (e[BUTTONS][32]==p.VR_BUTTON_IS_DOWN):
				controllerId = e[CONTROLLER_ID]
			if (controllerId == uiControllerId):
				controllerId = -1

	print("Using controllerId="+str(controllerId))
	global plugin
	print("current plugin=",plugin)
	#if (plugin>=0):
	#	print("unloading plugin")
	print("PluginId=",plugin)
	print("controllerId=",controllerId)
	print("pr2_cid=",pr2_cid)
	print("pr2_cid2=",pr2_cid2)
	p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid, pr2_cid2,pr2_gripper],[50,1])


resetAll()
print("after first resetAll, plugin=",plugin)
context=Context()

vrSyncPluginNeedsUpdate = True

mult = 1000.

logId = -1
logFileCount=0

while (1):
	t0 = mult*time.clock()
	frameNr=frameNr+1
	time.sleep(0.003)

	t1 = mult*time.clock()
	
	#print("numbodies=",p.getNumBodies())
	
	if (update_all_demos):
		for demo in demos:
			demo[2].update(context)
	else:
		demos[currentDemo][2].update(context)
	
	t2 = mult*time.clock()
	
	keys = p.getKeyboardEvents()
	
	
	for key in keys:
		if (keys[key]&p.KEY_WAS_TRIGGERED):
			print(keys[key])
			if key == p.B3G_RIGHT_ARROW:
				vr_shift[0] = vr_shift[0]+0.1
			if key == p.B3G_LEFT_ARROW:
				vr_shift[0] = vr_shift[0]-0.1
			if key == p.B3G_UP_ARROW:
				vr_shift[1] = vr_shift[1]+0.1
			if key == p.B3G_DOWN_ARROW:
				vr_shift[1] = vr_shift[1]-0.1
			if key == p.B3G_PAGE_UP:
				vr_shift[2] = vr_shift[2]+0.1
			if key == p.B3G_PAGE_DOWN:
				vr_shift[2] = vr_shift[2]-0.1
			#if key==ord('p'):
			#	logFileName ="profile%d.json" % logFileCount;
			#	print("Start Logging: %s" % logFileName)
			#	logFileCount += 1
			#	logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, logFileName)
				
			print("vr_shift",vr_shift)
		#if (keys[key]&p.KEY_WAS_RELEASED):
		#	if (logId>=0):
		#		print("Stop logging: %s [%d]" % (logFileName, logId) )
		#		p.stopStateLogging(logId)
		#		logId=-1
	t3 = mult*time.clock()
	
	pos = demos[currentDemo][1][3]
	rootPos = [pos[0],pos[1],pos[2]]
	rootPos[0] = rootPos[0] + vr_shift[0]
	rootPos[1] = rootPos[1] + vr_shift[1]
	rootPos[2] = rootPos[2] + vr_shift[2]
	p.setVRCameraState(rootPosition=rootPos)
	
	#uiPts = p.getClosestPoints(bodyA=uiCube,bodyB=pr2_gripper,distance=0.01, linkIndexA=0)
	#uiPts = p.getContactPoints(bodyA=uiCube,bodyB=pr2_gripper, linkIndexA=0)
	
	t4 = mult*time.clock()
	
	t5 = mult*time.clock()
		
	
	#keep the gripper centered/symmetric
	b = p.getJointState(pr2_gripper,2)[0]
	p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=3)
	
	t6 = mult*time.clock()
	
	events = p.getVREvents(allAnalogAxes=1)
	
	t7 = mult*time.clock()
	
	#print ("len events=",len(events))
	for e in (events):
		if e[CONTROLLER_ID] == uiControllerId:
			
			context.validPos = True
			context.pos = e[POSITION]
			context.orn = p.getQuaternionFromEuler([0,-math.pi,0]) #e[ORIENTATION]
			context.analog = e[ANALOG]
			
		
			p.resetBasePositionAndOrientation(uiCube,e[POSITION], e[ORIENTATION])
			if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED ):
				currentDemo = currentDemo+1
				if (currentDemo>=len(demos)):
					currentDemo = 0
				resetAll()
				
				vrSyncPluginNeedsUpdate = True
				
		if False:
			pos = e[POSITION]
			orn = e[ORIENTATION]
			lineFrom = pos
			mat = p.getMatrixFromQuaternion(orn)
			dir = [mat[0],mat[3],mat[6]]
			to = [pos[0]+dir[0]*rayLen,pos[1]+dir[1]*rayLen,pos[2]+dir[2]*rayLen]
			hit = p.rayTest(lineFrom,to)
			oldRay = pointRay
			color = [1,1,0]
			width = 3
			#pointRay = p.addUserDebugLine(lineFrom,to,color,width,lifeTime=1)
			#if (oldRay>=0):
			#	p.removeUserDebugItem(oldRay)
							
			if (hit):
				
				#if (hit[0][0]>=0):
				hitObjectUid = hit[0][0]
				linkIndex =  hit[0][1]
				if (hitObjectUid>=0):
					objectInfo = str(hitObjectUid)+" Link Index="+str(linkIndex)+"\nBase Name:"+p.getBodyInfo(hitObjectUid)[0].decode()+"\nBody Info:"+p.getBodyInfo(hitObjectUid)[1].decode()
					
					if (e[BUTTONS][33]&p.VR_BUTTON_WAS_TRIGGERED ):
						p.setVRCameraState(rootPosition=hit[0][3])
						print("button:",b)
						print("!!! new setVRCameraState at", hit[0][3])
							
				else:
					objectInfo="None"
			
		
			
		if e[CONTROLLER_ID] == controllerId:  # To make sure we only get the value for one of the remotes
			context.allAnalog=e[8]
			gpos,gorn = p.getBasePositionAndOrientation(pr2_gripper)
			maxDist = 0.2

			gripperLost = math.fabs(gpos[0]-e[POSITION][0]) > maxDist or math.fabs(gpos[1]-e[POSITION][1]) > maxDist or math.fabs(gpos[2]-e[POSITION][2]) > maxDist
			if (gripperLost):
				print("reset gripper")
				p.resetBasePositionAndOrientation(pr2_gripper,e[POSITION], e[ORIENTATION])

			if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED ):
				demos[currentDemo][2].reset()
				t = datetime.now()
				formatted_time = t.strftime('%d-%m-%y-%H%M%S')
				p.saveWorld("demoObjects%s.py" %formatted_time)
			
			
			
			#if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED ):
			#	mat = p.getMatrixFromQuaternion(gorn)
			#	cubepos = [gpos[0]+mat[0]*0.1,gpos[1]+mat[3]*0.1,gpos[2]+mat[6]*0.1]
			#	
			#	p.loadURDF("jenga/jenga.urdf",cubepos,gorn, useMaximalCoordinates=useMaximalCoordinatesEnvObjects)
			#	t = datetime.now()
			#	formatted_time = t.strftime('%d-%m-%y-%H%M')
			#	p.saveWorld("demoObjects%s.py" %formatted_time)
			
			if (vrSyncPluginNeedsUpdate):
				demos[currentDemo][2].reset()
				for i in range (numLines):
					p.removeUserDebugItem(lines[i])
					spacing = 0.01
					textPos = [0.08-(i+1)*spacing,.08,0.011]
					text = "Google Brain Robotics\nhttp://pybullet.org\n\nDemo:"+demos[currentDemo][0]#+" Frame:"+str(frameNr)+"\nObject UID:"+objectInfo
					textUid = p.addUserDebugText(text,textColorRGB=[0,0,0], textSize = 0.02, textPosition = textPos,textOrientation = textOrn,parentObjectUniqueId=uiCube, parentLinkIndex = -1)
					lines[i] = textUid
					
				p.setRealTimeSimulation(demos[currentDemo][3][0])
				#p.setTimeStep(demos[currentDemo][3][1])
				#p.setPhysicsEngineParameter(numSolverIterations=demos[currentDemo][3][2])
				
				print("vrSyncPluginNeedsUpdate other")
				vrSyncPluginNeedsUpdate = False
				p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid, pr2_cid2,pr2_gripper],[50,1])
					
			
				
				
		#		
		#	p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=500)
		#	relPosTarget = 1 - e[ANALOG]
		#	#open/close the gripper, based on analogue
		#	p.changeConstraint(pr2_cid2,gearRatio=1, erp=1, relativePositionTarget=relPosTarget, maxForce=1)
	
	t8 = mult*time.clock()
	#print("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f"%(t1-t0,t2-t1,t3-t2,t4-t3,t5-t4,t6-t5, t7-t6, t8-t7))		
	
	
			