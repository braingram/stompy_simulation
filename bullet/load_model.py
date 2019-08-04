#!/usr/bin/env python3
import sys

import pybullet as p
import pybullet_data

model_fn = 'stompy.urdf'
if len(sys.argv) > 1:
    model_fn = sys.argv[1]

c = p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
if 'sdf' in model_fn:
    rid = p.loadSDF(model_fn)
elif 'urdf' in model_fn:
    cubeStartPos = [0,0,1]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    rid = p.loadURDF(model_fn,cubeStartPos, cubeStartOrientation)
p.setRealTimeSimulation(True)
#p.stepSimulation()
nj = p.getNumJoints(rid)
# store joint ids for all leg joints
legs = {}
print("N joints: %s" % (nj, ))
for ji in range(nj):
    print("Joint %i" % (ji, ))
    info = p.getJointInfo(rid, ji)
    name = info[1].decode('ascii')
    ts = name.split('_')
    if ts[2] == 'body':  # hip joint
        ln = ts[4]
        jn = 'hip'
    else:
        ln = ts[2]
        if ts[-1] == 'thigh':
            jn = 'thigh'
        elif ts[-1] == 'upper':
            jn = 'knee'
        elif ts[-1] == 'lower':
            jn = 'calf'
        else:
            jn = None
    print("\t%s" % (info, ))
    if jn is None:
        continue
    if ln not in legs:
        legs[ln] = {}
    legs[ln][jn] = ji
print("")
print("Joint IDs:", legs)

# move hips
i = input("any key to move knees")
jids = [legs[ln]['knee'] for ln in legs]
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[-0.5] * len(jids))
i = input("any key to move knees")
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[-0.0] * len(jids))

i = input("any key to move thighs")
jids = [legs[ln]['thigh'] for ln in legs]
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[0.5] * len(jids))
i = input("any key to move thighs")
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[0.0] * len(jids))

i = input("any key to move hips")
jids = [legs[ln]['hip'] for ln in legs]
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[0.5] * len(jids))
i = input("any key to move hips")
p.setJointMotorControlArray(
    rid, jids, p.POSITION_CONTROL, targetPositions=[0.0] * len(jids))

# move thighs
# move knees

i = input("any key to exit")
# move joints with setJointMotorControl2/Array
#p.setJointMotorControlArray(
#    rid, indices, p.POSITION_CONTROL,
#    targetPositions=[0.1] * 6
p.disconnect()
