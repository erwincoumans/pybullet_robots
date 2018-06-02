import pybullet as p
import time
p.connect(p.GUI)
#p.loadSDF("f10_racecar/barca_track/barca_track.sdf")
p.loadSDF("f10_racecar/meshes/barca_track.sdf", globalScaling=1)
p.loadURDF("f10_racecar/racecar_differential.urdf", [0,0,1])
while (1):
	p.stepSimulation()
	time.sleep(1./240.)