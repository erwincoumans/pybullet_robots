import pybullet as p
import minitaur_demo
import kuka_demo

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
	
p.resetSimulation()
#waitId = p.addUserDebugText(text="Wait while loading SDF",textPosition=[0,0,0],textColorRGB=[1,0,0],textSize=10)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
objs = p.loadSDF("botlab/botlab.sdf")

zero=[0,0,0]
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

#p.removeUserDebugItem(waitId)

print("converting y to z axis")
for o in objs:
	pos,orn = p.getBasePositionAndOrientation(o)
	y2x = p.getQuaternionFromEuler([3.14/2.,0,0])
	newpos,neworn = p.multiplyTransforms(zero,y2x,pos,orn)
	p.resetBasePositionAndOrientation(o,newpos,neworn)
print("done")

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)


minitaur_demo_instance = minitaur_demo.MinitaurDemo()
kuka_demo_instance = kuka_demo.KukaDemo()

currentDemo = 0
demos=[	["minitaur", [-2.978418391524483, -4.541674828592511, 0.02539077526503508],minitaur_demo_instance],
						["kuka", [-3.272863154866866, 6.372032452458131, -0.013300959129968426],kuka_demo_instance],
					]

p.setRealTimeSimulation(1)
frame = 0

while(1):
	for demo in demos:
		demo[2].update()
		
	time.sleep(1./240.)
	
	keys = p.getKeyboardEvents()
	#if ord('k') in keys:
	#	print("k")
	for k in keys:
		if (keys[k]&p.KEY_WAS_TRIGGERED):
			print(keys[k])
			if k == p.B3G_RIGHT_ARROW:
				print("next demo:",frame)
				currentDemo = currentDemo+1
				if (currentDemo>=len(demos)):
					currentDemo=0
				print("currentDemo=",currentDemo)
				curCam = p.getDebugVisualizerCamera()
				print(curCam)
				
			if k == p.B3G_LEFT_ARROW:
				print("reset demo:",frame)
				demos[currentDemo][2].reset()
	p.setGravity(0,0,-10)
	frame=frame+1