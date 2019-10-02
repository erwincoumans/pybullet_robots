import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from pybullet_utils import bullet_client


import time




# Importing the libraries
import os
import time
import multiprocessing as mp
from multiprocessing import Process, Pipe

def createInstance(p, base_offset, useMaximalCoordinates, flags):
  
  
  
  objects = [
      p.loadURDF("kuka_iiwa/model_vr_limits.urdf", [base_offset[0]+1.400000, base_offset[1]+-0.200000,  base_offset[2]+0.600000], 
      [0.000000, 0.000000,0.000000, 1.000000], flags=flags)
  ]
  kuka = objects[0]
  jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
  for jointIndex in range(p.getNumJoints(kuka)):
    p.resetJointState(kuka, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
  
  objects = [
      p.loadURDF("lego/lego.urdf",[ base_offset[0]+1.000000, base_offset[1]+-0.200000, base_offset[2]+0.700000], [0.000000, 0.000000, 0.000000,
                 1.000000],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  objects = [
      p.loadURDF("lego/lego.urdf",[ base_offset[0]+1.000000, base_offset[1]+-0.200000, base_offset[2]+0.800000],[ 0.000000, 0.000000, 0.000000,
                 1.000000],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  objects = [
      p.loadURDF("lego/lego.urdf", [base_offset[0]+1.000000, base_offset[1]+-0.200000, base_offset[2]+0.900000], [0.000000, 0.000000, 0.000000,
                 1.000000],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  
  
  objects = [
      p.loadURDF("jenga/jenga.urdf", [base_offset[0]+1.300000,base_offset[1]+ -0.700000, base_offset[2]+0.750000],[ 0.000000, 0.707107, 0.000000,
                 0.707107],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  objects = [
      p.loadURDF("jenga/jenga.urdf", [base_offset[0]+1.200000, base_offset[1]+-0.700000, base_offset[2]+0.750000], [0.000000, 0.707107, 0.000000,
                 0.707107],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]

  objects = [
      p.loadURDF("table/table.urdf", [base_offset[0]+1.000000,base_offset[1]+ -0.200000,base_offset[2]+ 0.000000],[ 0.000000, 0.000000, 0.707107,
                 0.707107],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  objects = [
      p.loadURDF("teddy_vhacd.urdf",[base_offset[0]+ 1.050000,base_offset[1]+ -0.500000,base_offset[2]+ 0.700000], [0.000000, 0.000000, 0.707107,
                 0.707107],flags=flags, useMaximalCoordinates=useMaximalCoordinates)
  ]
  return kuka



_RESET = 1
_CLOSE = 2
_EXPLORE = 3


def ExploreWorker(rank, num_processes, childPipe, args):
  print("hi:",rank, " out of ", num_processes)  
  import pybullet as op
  import pybullet_data as pd
  logName=""
  p1=0
  n = 0
  while True:
    n += 1
    try:
      # Only block for short times to have keyboard exceptions be raised.
      if not childPipe.poll(0.001):
        continue
      message, payload = childPipe.recv()
    except (EOFError, KeyboardInterrupt):
      break
    if message == _RESET:
      op.connect(op.DIRECT)
      p1 = bullet_client.BulletClient()
      p1.setAdditionalSearchPath(pd.getDataPath())
      logName = str("batchsim")+str(rank)
      p1.loadURDF("plane.urdf")
      createInstance(p1, [0,0,0],useMaximalCoordinates=True, flags=0)
      p1.setPhysicsEngineParameter(minimumSolverIslandSize=100)
      p1.setGravity(0, 0, -10)
      childPipe.send(["reset ok"])
      logId = op.startStateLogging(op.STATE_LOGGING_PROFILE_TIMINGS,logName)
      continue
    if message == _EXPLORE:
      sum_rewards=rank

      
      numSteps = int(5000/num_processes)
      for i in range (numSteps):
        p1.stepSimulation()
      print("logId=",logId)
      print("numSteps=",numSteps)
      

      childPipe.send([sum_rewards])
      continue
    if message == _CLOSE:
      op.stopStateLogging(logId)
      childPipe.send(["close ok"])
      break
  childPipe.close()
  

if __name__ == "__main__":
  mp.freeze_support()
  num_processes = 16 
  processes = []
  args=[0]*num_processes
  
  childPipes = []
  parentPipes = []

  for pr in range(num_processes):
    parentPipe, childPipe = Pipe()
    parentPipes.append(parentPipe)
    childPipes.append(childPipe)

  for rank in range(num_processes):
    p = mp.Process(target=ExploreWorker, args=(rank, num_processes, childPipes[rank],  args))
    p.start()
    processes.append(p)
      
  
  for parentPipe in parentPipes:
    parentPipe.send([_RESET, "blaat"])
    
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    
    print("reset msg=",parentPipes[k].recv()[0])
  
  for parentPipe in parentPipes:
    parentPipe.send([_EXPLORE, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    positive_rewards[k] = parentPipes[k].recv()[0]
    print("positive_rewards=",positive_rewards[k])

  
  for parentPipe in parentPipes:
    parentPipe.send([_EXPLORE, "blaat"])
  
  positive_rewards = [0]*num_processes
  for k in range(num_processes):
    positive_rewards[k] = parentPipes[k].recv()[0]
    print("positive_rewards=",positive_rewards[k])

  
  for parentPipe in parentPipes:
    parentPipe.send([_CLOSE, "pay2"])
  
  for p in processes:
    p.join()
  
  #now we merge the separate json files into a single one
  fnameout = 'batchsim.json'
  count = 0
  outfile = open(fnameout, "w+")
  outfile.writelines(["{\"traceEvents\":[\n"])
  numFiles = num_processes
  for num in range(numFiles):
    print("num=",num)
    fname = 'batchsim%d_0.json' % (num)
    with open(fname) as infile:
        for line in infile:
          if "pid" in line:
            line = line.replace('\"pid\":1', '\"pid\":'+str(num))
            if num < (numFiles-1) and not "{}}," in line:
              line = line.replace('{}}', '{}},')
              print("line[",count,"]=",line)
            outfile.write(line)
          count += 1
  print ("count=",count)
  outfile.writelines(["],\n"])
  outfile.writelines(["\"displayTimeUnit\": \"ns\"}\n"])
  outfile.close()
