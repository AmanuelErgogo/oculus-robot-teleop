import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
# import xarm_sim
import xarm_heart

options = ""#"--mp4=\"video.mp4\""

p.connect(p.GUI, options=options)
p.setAdditionalSearchPath(pd.getDataPath())

timeStep=1./60.
p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)
 
xarm = xarm_heart.XArm7Sim(p,[0,0,0])
for i in range(330):
	xarm.step(i)
	p.stepSimulation()
	time.sleep(timeStep)
	
