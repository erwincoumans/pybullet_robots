import pybullet as p
import minitaur_demo
import kuka_demo

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)
	
p.resetSimulation()
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
objs = p.loadSDF("botlab/botlab.sdf")

zero=[0,0,0]
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)


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
demos=[	["pendulum", 
					[1.0,-92.0000991821289,-22.20002555847168,[0.014351863414049149, 0.39431828260421753, -0.8194975852966309]],
					kuka_demo_instance],
				["minitaur", 
					[1.0,-32.00006103515625,-15.800018310546875,[-1.2958829402923584, -1.0237634181976318, -0.6982248425483704]],
					minitaur_demo_instance],
			  ["kuka", 
						[1.0,151.1998291015625,-18.599998474121094,[2.127235174179077, 1.6416270732879639, -0.8124083876609802]],
						kuka_demo_instance],
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
				p.resetDebugVisualizerCamera(demos[currentDemo][1][0],
					demos[currentDemo][1][1],
					demos[currentDemo][1][2],
					demos[currentDemo][1][3])
				
			if k == p.B3G_LEFT_ARROW:
				print("reset demo:",frame)
				demos[currentDemo][2].reset()
				curCam = p.getDebugVisualizerCamera()
				print("curCam dist, yaw, pitch, targetPos",curCam[10],curCam[8],curCam[9], curCam[11])

	p.setGravity(0,0,-10)
	frame=frame+1