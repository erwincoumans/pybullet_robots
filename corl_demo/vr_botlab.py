import pybullet as p
import time
import math
import copy
import minitaur_demo
import kuka_demo
import pendulum_demo

vr_shift=[-1,-0.6,-1]

useMaximalCoordinatesEnvObjects=False #there is some issue with maximal coordinate bodies

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
p.resetSimulation()

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)


sphereRadius = 0.02
meshScale = [.1,.1,.01]
shift = [0,0,0]
colBoxId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[sphereRadius,sphereRadius,sphereRadius])
link_Masses=[1]
linkCollisionShapeIndices=[colBoxId]
linkVisualShapeIndices=[-1]
linkPositions=[[0,0,0.02]]
linkOrientations=[[0,0,0,1]]
linkInertialFramePositions=[[0,0,0]]
linkInertialFrameOrientations=[[0,0,0,1]]
linkIndices=[0]
linkJointTypes=[p.JOINT_FIXED]
linkJointAxis=[[0,0,1]]

objects = [p.loadURDF("sphere_small.urdf", [-1.447238,0.040553,-1.375914],[0.744341,-0.356588,-0.508423,0.245575], useMaximalCoordinates=useMaximalCoordinatesEnvObjects)]
objects = [p.loadURDF("sphere_small.urdf", [-1.337466,0.789305,-1.381118],[-0.061649,0.766884,0.468601,-0.434168], useMaximalCoordinates=useMaximalCoordinatesEnvObjects)]
objects = [p.loadURDF("sphere_small.urdf", [4.705443,2.788506,-1.319134],[0.830352,-0.126438,-0.528531,0.123221], useMaximalCoordinates=useMaximalCoordinatesEnvObjects)]
objects = [p.loadURDF("sphere_small.urdf", [4.645979,2.714498,-1.320512],[0.837715,0.056015,-0.458430,-0.291440], useMaximalCoordinates=useMaximalCoordinatesEnvObjects)]
objects = [p.loadURDF("sphere_small.urdf", [4.661952,3.834003,-1.363581],[0.731792,-0.049574,-0.270830,0.623437], useMaximalCoordinates=useMaximalCoordinatesEnvObjects)]



#objects = p.loadSDF("stadium.sdf")

visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="marble_cube.obj", rgbaColor=[1,1,1,1], specularColor=[1,1,1], visualFramePosition=shift, meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="marble_cube.obj", collisionFramePosition=shift,meshScale=meshScale)
#collisionShapeId = -1
uiCube = p.createMultiBody(baseMass=0,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = [0,1,0], linkMasses=link_Masses,linkCollisionShapeIndices=linkCollisionShapeIndices,linkVisualShapeIndices=linkVisualShapeIndices,linkPositions=linkPositions,linkOrientations=linkOrientations,linkInertialFramePositions=linkInertialFramePositions, linkInertialFrameOrientations=linkInertialFrameOrientations,linkParentIndices=linkIndices,linkJointTypes=linkJointTypes,linkJointAxis=linkJointAxis, useMaximalCoordinates=False)

p.changeVisualShape(uiCube,0,rgbaColor=[0,0,0,1])

textOrn = p.getQuaternionFromEuler([0,0,-1.5707963])
numLines = 1
lines = [-1]*numLines

p.stepSimulation()
#disable rendering during loading makes it much faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)


#p.addUserDebugLine([0,0,0],[1,0,0],[1,0,0],lineWidth=2)
#p.addUserDebugLine([0,0,0],[0,1,0],[0,1,0],lineWidth=2)
#p.addUserDebugLine([0,0,0],[0,0,1],[0,0,1],lineWidth=2)

#objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
#kitchenObj = p.loadSDF("kitchens/1.sdf")
botlabobjects = p.loadSDF("botlab/botlab.sdf", globalScaling=2, useMaximalCoordinates=useMaximalCoordinatesEnvObjects)
#botlabobjects = p.loadSDF("botlab/newsdf.sdf", globalScaling=2)
print("num botlabobjects  = ", botlabobjects )

for o in botlabobjects:
	pos,orn = p.getBasePositionAndOrientation(o)
	zero=[0,0,0]#[0,1,2.2]
	y2x = p.getQuaternionFromEuler([3.14/2.,0,-0])
	newpos,neworn = p.multiplyTransforms(zero,y2x,pos,orn)
	p.resetBasePositionAndOrientation(o,newpos,neworn)
	
#objects = [p.loadURDF("samurai.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("pr2_gripper2.urdf", 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000)]
pr2_gripper = objects[0]
print ("pr2_gripper=")
print (pr2_gripper)

jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000 ]
for jointIndex in range (p.getNumJoints(pr2_gripper)):
	p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex])
	p.setJointMotorControl2(pr2_gripper,jointIndex,p.POSITION_CONTROL,targetPosition=0,force=0)

pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])#[0.500000,0.300006,0.200000])
print ("pr2_cid")
print (pr2_cid)

pr2_cid2 = p.createConstraint(pr2_gripper,0,pr2_gripper,2,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
p.changeConstraint(pr2_cid2,gearRatio=1, erp=0.5, relativePositionTarget=0.5, maxForce=3)


r2d2 = p.loadURDF("r2d2.urdf", 3.000000,-0.200000,0.700000,0.000000,0.000000,0.000000,1.000000)
for i in range(p.getNumJoints(r2d2)):
	print(p.getJointInfo(r2d2,i))
p.setJointMotorControl2(r2d2,13,p.VELOCITY_CONTROL,targetVelocity=3)
for i in [2,3,6,7]:
	p.setJointMotorControl2(r2d2,i,p.VELOCITY_CONTROL,targetVelocity=-2)

#objects = [p.loadURDF("teddy_vhacd.urdf", 1.050000,-0.500000,0.700000,0.000000,0.000000,0.707107,0.707107)]
#objects = [p.loadURDF("cube_small.urdf", 0.950000,-0.100000,0.700000,0.000000,0.000000,0.707107,0.707107)]
#objects = [p.loadURDF("sphere_small.urdf", 0.850000,-0.400000,0.700000,0.000000,0.000000,0.707107,0.707107)]
#objects = [p.loadURDF("duck_vhacd.urdf", 0.850000,-0.400000,0.900000,0.000000,0.000000,0.707107,0.707107)]

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


p.setGravity(0,0,-10)

##show this for 10 seconds
#now = time.time()
#while (time.time() < now+10):
#	p.stepSimulation()
p.setRealTimeSimulation(1)

CONTROLLER_ID = 0
POSITION=1
ORIENTATION=2
ANALOG=3
BUTTONS=6

uiControllerId = -1

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



controllerId = -1

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

once = 0

p.configureDebugVisualizer(p.COV_ENABLE_VR_RENDER_CONTROLLERS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_VR_PICKING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_VR_TELEPORTING, 0)


if (once):
	logId = -1#p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "userDebugItems1.json")
	print("logId userdebug")
	print(logId)
	
	for i in range (numLines):
		spacing = 0.01
		textPos = [.1-(i+1)*spacing,.1,0.011]
		text = "ABCDEFGH"*10
		lines[i] = p.addUserDebugText(text,textColorRGB=[0,0,0], textSize = 0.01, textPosition = textPos,textOrientation = textOrn,parentObjectUniqueId=uiCube, parentLinkIndex = -1)
		
	if (once):
		once = 0
		if (logId and logId>0):
			p.stopStateLogging(logId)

frameNr = 0
objectInfo = ""
pointRay = -1
rayLen = 100
buttonA = 0


class EmptyDemo():
	def __init__(self):
		pass
		
	def reset(self):
		print ("racecar reset")
		
	def update(self, context):
		pass
		#print ("update")

minitaur_demo_instance = minitaur_demo.MinitaurDemo()
#minitaur_demo_instance = EmptyDemo()#minitaur_demo.MinitaurDemo()
kuka_demo_instance = kuka_demo.KukaDemo()
pendulum_demo_instance = EmptyDemo()#pendulum_demo.PendulumDemo()



currentDemo = 0
demos=[	["pendulum",
		 [1.0,-448.40008544921875, -11.000036239624023, [-1.0783566236495972, 1.9088447690010071, -1.105987787246704]],
		 pendulum_demo_instance],
				["minitaur",
					[1.0099999904632568,-34.40010452270508,4.5999579429626465, [-2.699556350708008, -2.7035043239593506, -1.1882244348526]],
					minitaur_demo_instance],
	   ["kuka",
		[1.0,163.59983825683594, -17.799997329711914, [4.711545467376709, 4.462226867675781, -1.0824081897735596]],
		kuka_demo_instance],
				]
				
p.setVRCameraState(rootPosition=demos[currentDemo][1][3])

plugin = p.loadPlugin("vrSyncPlugin")
print("PluginId="+str(plugin))
p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid, pr2_cid2,pr2_gripper],[50,1])
t7=0

class Context():
	def __init__(self):
		self.validPos = False
		self.vrMode = True
		self.pos = [0,0,0]
  	
context=Context()


mult = 1000.
while (1):
	t0 = mult*time.clock()
	frameNr=frameNr+1
	time.sleep(0.001)

	t1 = mult*time.clock()
	
	for demo in demos:
		demo[2].update(context)
	
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
		print("vr_shift",vr_shift)
				
	t3 = mult*time.clock()
	
	rootPos = copy.deepcopy(demos[currentDemo])[1][3]
	rootPos[0] = rootPos[0] + vr_shift[0]
	rootPos[1] = rootPos[1] + vr_shift[1]
	rootPos[2] = rootPos[2] + vr_shift[2]
	p.setVRCameraState(rootPosition=rootPos)
	
	#uiPts = p.getClosestPoints(bodyA=uiCube,bodyB=pr2_gripper,distance=0.01, linkIndexA=0)
	uiPts = p.getContactPoints(bodyA=uiCube,bodyB=pr2_gripper, linkIndexA=0)
	
	t4 = mult*time.clock()
	
	if (len(uiPts)):
		if (buttonA!=1):
			buttonA = 1
			p.changeVisualShape(uiCube,0,rgbaColor=[1,1,0,1])
			demos[currentDemo][2].reset()
			p.saveWorld("demoObjects.py")
	

			
	else:
		if (buttonA!=0):
			p.changeVisualShape(uiCube,0,rgbaColor=[0,0,0,1])
			buttonA = 0
	
	t5 = mult*time.clock()
		
	#if ((frameNr % 32) == 0):
	#	p.removeUserDebugItem(lines[i])
	#	for i in range (numLines):
	#		spacing = 0.01
	#		textPos = [.1-(i+1)*spacing,.1,0.011]
	#		text = "Demo:"+demos[currentDemo][0]+" Frame:"+str(frameNr)+"\nObject UID:"+objectInfo
	#		textUid = p.addUserDebugText(text,textColorRGB=[0,0,0], textSize = 0.02, textPosition = textPos,textOrientation = textOrn,parentObjectUniqueId=uiCube, parentLinkIndex = -1)
	#		lines[i] = textUid
	
	
	#keep the gripper centered/symmetric
	b = p.getJointState(pr2_gripper,2)[0]
	p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=3)
	
	t6 = mult*time.clock()
	
	events = p.getVREvents()
	
	t7 = mult*time.clock()
	
	#print ("len events=",len(events))
	for e in (events):
		if e[CONTROLLER_ID] == uiControllerId:
			p.resetBasePositionAndOrientation(uiCube,e[POSITION], e[ORIENTATION])
			if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED ):
				currentDemo = currentDemo+1
				if (currentDemo>=len(demos)):
					currentDemo = 0
		
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
			
			gpos,gorn = p.getBasePositionAndOrientation(pr2_gripper)
			
			
			if (e[BUTTONS][32]&p.VR_BUTTON_WAS_TRIGGERED ):
				p.loadURDF("sphere_small.urdf",gpos,gorn, useMaximalCoordinates=useMaximalCoordinatesEnvObjects)
			
			#sync the vr pr2 gripper with the vr controller position
			
			
			if (demos[currentDemo][0] == "kuka"):
					context.validPos = True
					context.pos = e[POSITION]
					context.orn = e[ORIENTATION]
					context.analog = e[ANALOG]
					p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid, pr2_cid2,pr2_gripper],[50,0])
					p.resetBasePositionAndOrientation(pr2_gripper,[0,0,-10], [0,0,0,1])
			else:
					context.validPos = False
					p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid, pr2_cid2,pr2_gripper],[50,1])
					maxDist = 0.2
					gripperLost = math.fabs(gpos[0]-e[POSITION][0]) > maxDist or math.fabs(gpos[1]-e[POSITION][1]) > maxDist or math.fabs(gpos[2]-e[POSITION][2]) > maxDist
					if (gripperLost):
						print("reset gripper")
						p.resetBasePositionAndOrientation(pr2_gripper,e[POSITION], e[ORIENTATION])
				
				
		#		
		#	p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=500)
		#	relPosTarget = 1 - e[ANALOG]
		#	#open/close the gripper, based on analogue
		#	p.changeConstraint(pr2_cid2,gearRatio=1, erp=1, relativePositionTarget=relPosTarget, maxForce=1)
	
	t8 = mult*time.clock()
	#print("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f"%(t1-t0,t2-t1,t3-t2,t4-t3,t5-t4,t6-t5, t7-t6, t8-t7))		
	
	
			