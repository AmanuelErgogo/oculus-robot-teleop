import time
import numpy as np
import math

useNullSpace = 1
useDynamics = 1
ikMaxNumIterations=50
ikSolver = 0
xarmEndEffectorIndex = 7
xarmNumDofs = 7 

ll = [-17]*xarmNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [17]*xarmNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [17]*xarmNumDofs
#restposes for null space
jointPositions=[0,0,0,0,0,0,0]
rp = jointPositions
global jointPoses
jointPoses = jointPositions

class XArm7BulletIk(object):
  def __init__(self, bullet_client, offset, initial_jointPositions=[0, 0, 0, 0, 0, 0, 0]):
    global jointPoses
    self.bullet_client = bullet_client
    self.offset = np.array(offset)
    self.jointPoses = [0]*xarmNumDofs
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    jointPoses = initial_jointPositions

    orn=[0,0,0,1]
    self.xarm = self.bullet_client.loadURDF("xarm7_robot.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    for j in range(self.bullet_client.getNumJoints(self.xarm)):
      self.bullet_client.changeDynamics(self.xarm, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.xarm, j)
  
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.xarm, j, initial_jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.xarm, j, initial_jointPositions[index]) 
        index=index+1
    self.t = -math.pi/4.0
    
  def reset(self):
    pass

  def step(self, pos, orn):
    global jointPoses
    if useNullSpace:
        restPoses = [0]*xarmNumDofs
        jointPoses = self.bullet_client.calculateInverseKinematics(self.xarm,xarmEndEffectorIndex, pos, orn,lowerLimits=ll, 
          upperLimits=ul,jointRanges=jr, restPoses=np.array(restPoses).tolist(),residualThreshold=1e-5, maxNumIterations=ikMaxNumIterations)
        self.jointPoses = jointPoses
    else:
        self.jointPoses = self.bullet_client.calculateInverseKinematics(self.xarm,xarmEndEffectorIndex, pos, orn, maxNumIterations=ikMaxNumIterations)
    if useDynamics:
      for i in range(xarmNumDofs):
          pose = self.jointPoses[i]
          self.bullet_client.setJointMotorControl2(self.xarm, i+1, self.bullet_client.POSITION_CONTROL, pose,force=5 * 240.)
    else:
      for i in range(xarmNumDofs):
        self.bullet_client.resetJointState(self.xarm, i+1, jointPoses[i])
    ls = self.bullet_client.getLinkState(self.xarm, xarmEndEffectorIndex, computeForwardKinematics=True)
    linkComPos=np.array(ls[0])
    linkComOrn=ls[1]
    linkUrdfPos=np.array(ls[4])
    linkUrdfOrn=ls[5]
    #print("linkComPos=",linkComPos)
    #print("linkUrdfOrn=",linkUrdfOrn)
    mat = self.bullet_client.getMatrixFromQuaternion(linkUrdfOrn)
    #print("mat=",mat)
    self.bullet_client.addUserDebugLine(pos, linkUrdfPos, [1,0,0],lifeTime=100)
    diff = linkUrdfPos-np.array(pos)

    return self.jointPoses
    #print("diff=",diff)
    
  
