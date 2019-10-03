import pybullet as p
import pybullet_data as pd
import math
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", [0,0,1], useFixedBase=True)

p.setGravity(0,0,0)#-9.8)
jointIds = []
paramIds = []

#jointPositions=[0,-0.5,0,-1.8,0,1.4,0.7,0.014,0.016]
jointPositions=[ 0.008948, 0.345998, -0.103308, -2.315629, 0.075034, 2.657321, 0.630666, 0.024,0.026]

legos=[]
#spawn a few duplo bricks
for i in range (3):
  
  legos.append(p.loadURDF("lego/lego.urdf", 0.000000, -0.200000, 0.6500000+i*0.03, 0.000000, 0.000000, 0.000000,1.000000))

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 8
#lower limits for null space (todo: set them to proper range)
ll = [-7]*pandaEndEffectorIndex
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaEndEffectorIndex
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaEndEffectorIndex
#restposes for null space
rp = jointPositions



posXid = p.addUserDebugParameter("basePosX",-1,1,-0.526)
posYid = p.addUserDebugParameter("basePosY",-1,1,0)
posZid = p.addUserDebugParameter("basePosZ",0,1,0.67)

useIkId = p.addUserDebugParameter("useIkId",0,1,0)
ikposXid = p.addUserDebugParameter("ikPosX",-1,1,0)
ikposYid = p.addUserDebugParameter("ikPosY",-1,1,-0.2)
ikposZid = p.addUserDebugParameter("ikPosZ",0,1,0.8)


index = 0
print("numJoints = ",p.getNumJoints(panda))
#allow to manually override the joint positions
for j in range(p.getNumJoints(panda)):
  p.changeDynamics(panda, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(panda, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -0.1, 0.1, jointPositions[index]))
    p.resetJointState(panda, j, jointPositions[index]) 
    index=index+1
  if (jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, jointPositions[index]))
    p.resetJointState(panda, j, jointPositions[index]) 
    index=index+1
    
while (1):
  posX = p.readUserDebugParameter(posXid)
  posY = p.readUserDebugParameter(posYid)
  posZ = p.readUserDebugParameter(posZid)
  p.resetBasePositionAndOrientation(panda,[posX,posY,posZ],[0,0,0,1])
  
  useIk = p.readUserDebugParameter(useIkId)
  if (useIk>0.5):
    pos=[0,-0.2,0.8]
    orn = [1,0,0,0]#p.getQuaternionFromEuler([-math.pi, 0,0])
    #s = p.getLinkState(panda,pandaEndEffectorIndex)
    #print("s=",s)
    #jointPoses = p.calculateInverseKinematics(panda,pandaEndEffectorIndex,
    #                                                pos,orn, solver=ikSolver)
    jointPoses = p.calculateInverseKinematics(panda,pandaEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
    for i in range(pandaEndEffectorIndex):#len(jointPoses)):
        #p.resetJointState(panda, i, jointPoses[i])                
        p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
  else:
    for i in range(len(paramIds)):
      c = paramIds[i]
      targetPos = p.readUserDebugParameter(c)
      p.setJointMotorControl2(panda, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
    
  p.stepSimulation()
  
  
  
 