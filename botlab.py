import pybullet as p

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
while(1):
	p.stepSimulation()

