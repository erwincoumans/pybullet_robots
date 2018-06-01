from time import sleep
import pybullet as p
import numpy as np



def setUpWorld(initialSimSteps=100):
    """
    Reset the simulation to the beginning and reload all models.

    Parameters
    ----------
    initialSimSteps : int

    Returns
    -------
    baxterId : int
    endEffectorId : int 
    """
    p.resetSimulation()

    
    # Load plane
    p.loadURDF("plane.urdf", [0, 0, -1], useFixedBase=True)

    sleep(0.1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
    # Load Baxter
    baxterId = p.loadURDF("baxter_common/baxter_description/urdf/toms_baxter.urdf", useFixedBase=True)
    p.resetBasePositionAndOrientation(baxterId, [0.5, -0.8, 0.0], [0., 0., -1., -1.])
    #p.resetBasePositionAndOrientation(baxterId, [0.5, -0.8, 0.0],[0,0,0,1])
    #p.resetBasePositionAndOrientation(baxterId, [0, 0, 0], )

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

    # Grab relevant joint IDs
    endEffectorId = 48 # (left gripper left finger)

    # Set gravity
    p.setGravity(0., 0., -10.)

    # Let the world run for a bit
    for _ in range(initialSimSteps):
        p.stepSimulation()

    return baxterId, endEffectorId

def getJointRanges(bodyId, includeFixed=False):
    """
    Parameters
    ----------
    bodyId : int
    includeFixed : bool

    Returns
    -------
    lowerLimits : [ float ] * numDofs
    upperLimits : [ float ] * numDofs
    jointRanges : [ float ] * numDofs
    restPoses : [ float ] * numDofs
    """

    lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []

    numJoints = p.getNumJoints(bodyId)

    for i in range(numJoints):
        jointInfo = p.getJointInfo(bodyId, i)

        if includeFixed or jointInfo[3] > -1:

            ll, ul = jointInfo[8:10]
            jr = ul - ll

            # For simplicity, assume resting state == initial state
            rp = p.getJointState(bodyId, i)[0]

            lowerLimits.append(-2)
            upperLimits.append(2)
            jointRanges.append(2)
            restPoses.append(rp)

    return lowerLimits, upperLimits, jointRanges, restPoses

def accurateIK(bodyId, endEffectorId, targetPosition, lowerLimits, upperLimits, jointRanges, restPoses, 
               useNullSpace=False, maxIter=10, threshold=1e-4):
    """
    Parameters
    ----------
    bodyId : int
    endEffectorId : int
    targetPosition : [float, float, float]
    lowerLimits : [float] 
    upperLimits : [float] 
    jointRanges : [float] 
    restPoses : [float]
    useNullSpace : bool
    maxIter : int
    threshold : float

    Returns
    -------
    jointPoses : [float] * numDofs
    """
    closeEnough = False
    iter = 0
    dist2 = 1e30

    numJoints = p.getNumJoints(baxterId)

    while (not closeEnough and iter<maxIter):
        if useNullSpace:
            jointPoses = p.calculateInverseKinematics(bodyId, endEffectorId, targetPosition,
                lowerLimits=lowerLimits, upperLimits=upperLimits, jointRanges=jointRanges, 
                restPoses=restPoses)
        else:
            jointPoses = p.calculateInverseKinematics(bodyId, endEffectorId, targetPosition)
    
        for i in range(numJoints):
            jointInfo = p.getJointInfo(bodyId, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.resetJointState(bodyId,i,jointPoses[qIndex-7])
        ls = p.getLinkState(bodyId,endEffectorId)    
        newPos = ls[4]
        diff = [targetPosition[0]-newPos[0],targetPosition[1]-newPos[1],targetPosition[2]-newPos[2]]
        dist2 = np.sqrt((diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]))
        print("dist2=",dist2)
        closeEnough = (dist2 < threshold)
        iter=iter+1
    print("iter=",iter)
    return jointPoses

def setMotors(bodyId, jointPoses):
    """
    Parameters
    ----------
    bodyId : int
    jointPoses : [float] * numDofs
    """
    numJoints = p.getNumJoints(bodyId)

    for i in range(numJoints):
        jointInfo = p.getJointInfo(bodyId, i)
        #print(jointInfo)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[qIndex-7])



if __name__ == "__main__":
    guiClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2., 180, 0., [0.52, 0.2, np.pi/4.])


    targetPosXId = p.addUserDebugParameter("targetPosX",-1,1,0.2)
    targetPosYId = p.addUserDebugParameter("targetPosY",-1,1,0)
    targetPosZId = p.addUserDebugParameter("targetPosZ",-1,1,-0.1)
    nullSpaceId = p.addUserDebugParameter("nullSpace",0,1,1)

    baxterId, endEffectorId = setUpWorld()

    lowerLimits, upperLimits, jointRanges, restPoses = getJointRanges(baxterId, includeFixed=False)

    
    #targetPosition = [0.2, 0.8, -0.1]
    #targetPosition = [0.8, 0.2, -0.1]
    targetPosition = [0.2, 0.0, -0.1]
    
    p.addUserDebugText("TARGET", targetPosition, textColorRGB=[1,0,0], textSize=1.5)


    maxIters = 100000

    sleep(1.)

    p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL )
    for _ in range(maxIters):
      p.stepSimulation()
      targetPosX = p.readUserDebugParameter(targetPosXId)
      targetPosY = p.readUserDebugParameter(targetPosYId)
      targetPosZ = p.readUserDebugParameter(targetPosZId)
      nullSpace = p.readUserDebugParameter(nullSpaceId)

      targetPosition=[targetPosX,targetPosY,targetPosZ]
      
      useNullSpace = nullSpace>0.5
      print("useNullSpace=",useNullSpace)
      jointPoses = accurateIK(baxterId, endEffectorId, targetPosition, lowerLimits, upperLimits, jointRanges, restPoses, useNullSpace=useNullSpace)
      setMotors(baxterId, jointPoses)

      #sleep(0.1)

