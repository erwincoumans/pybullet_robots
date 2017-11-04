import pybullet as p
p.connect(p.GUI)
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
while(1):
	p.stepSimulation()

