import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import panda_sim_grasp as panda_sim

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
p.setAdditionalSearchPath(pd.getDataPath())

timeStep=1./120.#240.
p.setTimeStep(timeStep)
p.setGravity(0,-9.8,0)

panda = panda_sim.PandaSimAuto(p,[0,0,0])
logId = panda.bullet_client.startStateLogging(panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
panda.bullet_client.submitProfileTiming("start")
for i in range (100000):
	panda.bullet_client.submitProfileTiming("full_step")
	panda.step()
	p.stepSimulation()
	time.sleep(timeStep)
	panda.bullet_client.submitProfileTiming()
panda.bullet_client.submitProfileTiming()
panda.bullet_client.stopStateLogging(logId)
	
