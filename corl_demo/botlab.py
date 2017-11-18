import pybullet as p
import minitaur_demo
import kuka_demo
import pendulum_demo

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
	p.connect(p.GUI)

p.resetSimulation()
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)

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
pendulum_demo_instance = pendulum_demo.PendulumDemo()

currentDemo = 0
demos=[	["pendulum",
		 [1.0,-448.40008544921875, -11.000036239624023, [-1.5783566236495972, 0.9088447690010071, -1.105987787246704]],
		 pendulum_demo_instance],
				["minitaur",
					[1.0099999904632568,-34.40010452270508,4.5999579429626465, [-2.699556350708008, -2.7035043239593506, -1.1882244348526]],
					minitaur_demo_instance],
	   ["kuka",
		[1.0,163.59983825683594, -17.799997329711914, [4.711545467376709, 4.462226867675781, -1.0824081897735596]],
		kuka_demo_instance],
				]

p.setRealTimeSimulation(1)
frame = 0
p.setGravity(0,0,-10)

p.resetDebugVisualizerCamera(demos[currentDemo][1][0],
											 demos[currentDemo][1][1],
											 demos[currentDemo][1][2],
											 demos[currentDemo][1][3])


class Context():
	def __init__(self):
		self.validPos = False
		self.vrMode = False
		self.pos = [0,0,0]
  	
context=Context()

while(1):
	for demo in demos:
		demo[2].update(context)

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


frame=frame+1
