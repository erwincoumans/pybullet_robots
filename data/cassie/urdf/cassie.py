import pybullet as p
p.connect(p.GUI)
p.loadURDF("cassie_collide.urdf")
while (1):
  p.stepSimulation()
