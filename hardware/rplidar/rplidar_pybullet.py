import pybullet as p
import time
import math
import time

from rplidar import RPLidar
PORT_NAME='/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)
time.sleep(1)
print("lidar info:", lidar.get_info())
numRays = 5000
iterator = lidar.iter_scans()

useGui = True

if (useGui):
	p.connect(p.GUI)
else:
	p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

#p.loadURDF("samurai.urdf")
#p.loadURDF("r2d2.urdf",[3,3,1])


rayFrom=[]
rayTo=[]
rayIds=[]

	
rayLen = 0.1


rayHitColor = [0,0,1]
rayMissColor = [0,1,0]

replaceLines = True
	
for i in range (numRays):
	rayFrom.append([0,0,1])
	rayTo.append([rayLen*math.sin(2.*math.pi*float(i)/numRays), rayLen*math.cos(2.*math.pi*float(i)/numRays),1])
	if (replaceLines):
		rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
	else:
		rayIds.append(-1)

if (not useGui):
	timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS,"rayCastBench.json")

numSteps = 100
if (useGui):
	numSteps = 100 
index = 0
totalRays = 0
frame = 0
while True:
	keys=p.getKeyboardEvents()
	if ord('q') in keys:
		break

	print("frame=",frame, "total rays=",totalRays)
	frame +=1
	p.stepSimulation()
	#for j in range (8):
	results = p.rayTestBatch(rayFrom,rayTo)

	scan=next(iterator)	
	
	if (useGui):
		if (not replaceLines):
			p.removeAllUserDebugItems()
			
		for i in range (len(scan)):
			#print(scan[i])
			hitObjectUid=results[i][0]
			angle = scan[i][1]
			#print("angle=",angle)
			length = scan[i][2]/1000. #in meters
			#print("length=",length)
			angleRad = (angle/360.)*(2*math.pi)
			fromPosition = [(length)*math.sin(angleRad),(length)*math.cos(angleRad),0]
			hitPosition = [length*math.sin(angleRad),length*math.cos(angleRad),0.1]
			p.addUserDebugLine(fromPosition,hitPosition, rayHitColor,replaceItemUniqueId=rayIds[index])
			index+=1
			totalRays+=1
			if (index>=numRays):
				index=0
	#time.sleep(1./240.)
if (not useGui):
	p.stopStateLogging(timingLog)

lidar.stop()
lidar.disconnect()
print("bye!")
