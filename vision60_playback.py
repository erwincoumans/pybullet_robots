from __future__ import print_function
import math
import pybullet as p
import pybullet
import time
import pybullet_data as pd

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
#obUids = p.loadMJCF("mjcf/humanoid.xml")
#humanoid = obUids[1]
plane = p.loadURDF("plane.urdf")
#URDF doesn't seem to have reliable inertia data so no flags=p.URDF_USE_INERTIA_FROM_FILE
humanoid = p.loadURDF("quadruped/vision60.urdf",[0,0,0.5],  useFixedBase=False)#True)
gravId = p.addUserDebugParameter("gravity",-10,10,-10)
jointIds=[]
paramIds=[]

p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(0.001)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

for j in range (p.getNumJoints(humanoid)):
  p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
  info = p.getJointInfo(humanoid,j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,0))

p.setRealTimeSimulation(0)#1)

skipCount = 40
count=0

numDoF=12
jointMap=[-1]*numDoF
dirs=[1]*numDoF
swaps=[0]*numDoF
jointMap[0]=8
jointMap[1]=0
jointMap[2]=1
swaps[2]=1
dirs[1]=-1
jointMap[3]=9
dirs[3]=-1
jointMap[4]=2
dirs[4]=-1
jointMap[5]=3
swaps[5]=1
jointMap[6]=10
jointMap[7]=4
jointMap[8]=5
swaps[8]=1
dirs[8]=-1
jointMap[9]=11
dirs[9]=-1
jointMap[10]=6
jointMap[11]=7
dirs[11]=-1
swaps[11]=1
skipOne = 1
prevt=-1
with open("newvision60_0.log","r") as filestream:
#with open("data1.txt","r") as filestream:
  for line in filestream:
  
    count+=1
    if count<skipCount:
      continue


    currentline = line.split(",")
    frame = currentline[0]
    t = int(currentline[1])

    joints=currentline[2:14]
    #for j in range (12):
    #        targetPos = float(joints[j])
    #        p.setJointMotorControl2(quadruped,jointIds[j],p.POSITION_CONTROL,jointDirections[j]*targetPos+jointOffsets[j], force=maxForce)

    p.setGravity(0,0,p.readUserDebugParameter(gravId))
    for i in range(12):
      c = paramIds[i]
      targetPos = p.readUserDebugParameter(c)
      if jointMap[i]>=0:
        #print("jointIds[",i,"]=",jointIds[i])
        targetPos = dirs[i]*float(joints[jointMap[i]])

        if swaps[i]:
          targetPos=math.pi/2.-targetPos

      p.setJointMotorControl2(humanoid,jointIds[i],p.POSITION_CONTROL,targetPos, force=120.)

    if prevt>=0:
      dt = t-prevt
      if dt<5:
        for i in range (dt):
          p.stepSimulation()
          time.sleep(1./1000.)
    prevt=t


