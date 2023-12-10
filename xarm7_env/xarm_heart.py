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

# jointPositions=[-0.1,-29.1,5.7,31.7,-0.2,44.2,0.3] 
# jointPositions=np.deg2rad(jointPositions)
jointPositions = [0, 0, 0, 0, 0, 0, 0]
rp = jointPositions
jointPoses = jointPositions
class XArm7Sim(object):
  def __init__(self, bullet_client, offset):

    self.bullet_client = bullet_client
    self.offset = np.array(offset)
    self.jointPoses = [0]*xarmNumDofs
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    legos=[]
    # self.bullet_client.loadURDF("tray/traybox.urdf", [0.4+offset[0], offset[1], offset[2]], [0,0,0,1], flags=flags)
    # legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.3, 0.1, 0.3])+self.offset, flags=flags))
    # legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.3, -0.1, 0.3])+self.offset, flags=flags))
    # legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.5, 0.1, 0.3])+self.offset, flags=flags))
    # sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.4, 0, 0.3])+self.offset, flags=flags)
    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.3, 0, 0.3])+self.offset, flags=flags)
    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.5, 0, 0.3])+self.offset, flags=flags)
    self.index = 0
    orn=[0,0,0,1]
    self.xarm = self.bullet_client.loadURDF("xarm7_robot.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    for j in range(self.bullet_client.getNumJoints(self.xarm)):
      self.bullet_client.changeDynamics(self.xarm, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.xarm, j)
  
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.xarm, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.xarm, j, jointPositions[index]) 
        index=index+1
    self.t = -math.pi/4.0
    self.num_points = 300
    self.t_heart = np.linspace(0, 2 * np.pi, self.num_points)
    self.i = 0
  def reset(self):
    pass

  def normalize(self, value, original_min=-20, original_max=20, new_min=0.25, new_max=0.55):
    normalized_value = ((value - original_min) * (new_max - new_min)) / (original_max - original_min) + new_min
    return normalized_value
  
  def generate_straight_line_path(self, start_point=[0.6, 0.051000000000000045, 0.44875000000000004], end_point=[0.6, 0.051000000000000045, 0.60], num_points=30):
    start_point = np.array(start_point)
    end_point = np.array(end_point)
    path_points = np.linspace(start_point, end_point, num_points)
    return path_points

  def points(self):
    point_list = self.generate_straight_line_path()
    self.index = self.index + 1
    return point_list[self.index][0], point_list[self.index][1], point_list[self.index][2]

  def step(self, i):
    offset = np.array([0, -0.349, 0])
    t = self.t
    self.t += 1./60.
    
    t_heart = self.t_heart[self.i]
    
    # pos = [0.407+self.offset[0]+0.2 * math.sin(1.5 * t), 0.0+self.offset[1]+0.2 * math.cos(1.5 * t), 0.12+self.offset[2]]
    #orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    orn = [0.9,0,0.75,0]

    # heart path
    y = 1.3*(16 * math.sin(t_heart)**3)
    z = 1.3*(13 * math.cos(t_heart) - 5 * math.cos(2*t_heart) - 2 * math.cos(3*t_heart) - math.cos(4*t_heart))
    x = 0.6 # 0.12+self.offset[2]

    # normalize
    y_norm = self.normalize(y) + offset[1]
    z_norm = self.normalize(z)

    pos = [ x, y_norm, z_norm ]
    # orn = [ -0.4561459, -0.7143862, 0.5303228, 0.018464 ]
    # orn = [ 0.3328656, 0.6329588, -0.3331887, -0.6144501 ]

    if i > self.num_points -1:
      xl, yl, zl = self.points()
      pos = [xl, yl, zl]

    # print(f"pos: {pos}")
    
    if useNullSpace:
        restPoses = [0]*xarmNumDofs
        start = time.time()
        jointPoses = self.bullet_client.calculateInverseKinematics(self.xarm,xarmEndEffectorIndex, pos, orn,lowerLimits=ll, 
          upperLimits=ul,jointRanges=jr, restPoses=np.array(restPoses).tolist(),residualThreshold=1e-5, maxNumIterations=ikMaxNumIterations)
        print("ik_with NullSpace time: ", time.time() - start)
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
    #print("diff=",diff)

    # update i
    
    if self.i ==  self.num_points - 1:
        self.i = 0
    else:
        self.i = self.i + 1
  
