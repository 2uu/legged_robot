#import os
#os.chdir('./') # urdfのあるところに移動
import pybullet_data
import pybullet as p
import random
import time
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
p.setTimeStep(1./500)
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

boxId = p.loadURDF("../urdf/legged_robot.urdf",cubeStartPos, cubeStartOrientation)

#print(p.getNumJoints(boxId))
#print([p.getJointInfo(boxId, i) for i in range(21)])
maxForce = 500
count = 0
p.getCameraImage(480,320)

while True: # とりあえずpythonを抜けないように
    p.stepSimulation()
    #cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    state = p.getLinkState(boxId, 1)
    euler = p.getEulerFromQuaternion(state[1])
    print(list(map(lambda x:x*180/3.14, euler)))
    #print(cubePos,cubeOrn)
    if count%3000 == 0:
        [p.setJointMotorControl2(bodyUniqueId=boxId, 
        jointIndex=i, 
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity = random.uniform(-0.1,0.1),
        force = maxForce) for i in range(21)]
    count += 1
p.disconnect()
