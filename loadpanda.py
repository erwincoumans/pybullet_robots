import pybullet as p
p.connect(p.GUI)
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.setGravity(0,0,-9.8)
jointIds = []
paramIds = []

jointPositions=[0,0,0,-1.4,0,1.4,0.71,0.014,0.016]

index = 0
print("numJoints = ",p.getNumJoints(panda))
for j in range(p.getNumJoints(panda)):
  p.changeDynamics(panda, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(panda, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -0.1, 0.1, jointPositions[index]))
    index=index+1
  if (jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, jointPositions[index]))
    index=index+1
    
while (1):
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(panda, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
  p.stepSimulation()
  p.saveWorld("bla")