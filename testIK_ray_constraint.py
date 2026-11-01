"""Full-body IK demo with a draggable world-space ray constraint.

Requires the existing libcalab_ogre3d Python API and the new
    ik._setRayConstraint(index, bone, local_point, ray_origin, ray_direction)
binding to the C++ IK solver.

The endpoint constraint is a RAY, not an infinite line: ray origin + t *
direction, t >= 0. Direction is a unit vector in world space.
"""

import math
import sys

import libcalab_ogre3d  
assert(libcalab_ogre3d  .__version__>='0.3.5')

from libcalab_ogre3d import m, lua, RE, control
from easydict import EasyDict as edict


config_hyunwoo = edict({
    'skel': 'work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl',
    'motion': 'work/taesooLib/Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof',
    'IKconfig': [
        {'effector': ['LeftKnee', 'LeftAnkle', m.vector3(0, -0.06, 0.08)], 'reversed': False},
        {'effector': ['RightKnee', 'RightAnkle', m.vector3(0, -0.06, 0.08)], 'reversed': False},
    ],
    'skinScale': 100,
    'initialHeight': 0,
})

config = config_hyunwoo
mCON = None


def eventFunction(ev, i):
    limbik()


def createIKsolver(loader, config):
    out = edict()
    effectors = m.Effectors()
    numCon = len(config)
    effectors.resize(numCon)
    out.effectors = effectors
    out.numCon = numCon

    for i in range(numCon):
        conInfo = config[i]['effector']
        effectors(i).init(loader.getBoneByName(conInfo[1]), conInfo[2])

    out.solver = m.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo)
    return out


def loadMotion(skel, motion, skinScale=None):
    mot = edict()
    mot.loader = RE.WRLloader(skel)
    mot.motionDOFcontainer = m.MotionDOFcontainer(mot.loader.dofInfo, motion)
    if skinScale is not None:
        mot.skin = RE.createVRMLskin(mot.loader, True)
        mot.skin.scale(skinScale, skinScale, skinScale)
        mot.skin.applyMotionDOF(mot.motionDOFcontainer.mot)
        mot.skin.setMaterial('lightgrey_transparent')
    return mot


def ctor(this):
    global mMot, mLoader, mMotionDOFcontainer, mMotionDOF
    global mSkin, mPose, mSolverInfo, mCON

    RE.viewpoint().setFOVy(45.000000)
    RE.viewpoint().setZoom(1.000000)
    RE.viewpoint().vpos.set(436.272692, 227.164791, 77.243196)
    RE.viewpoint().vat.set(-16.574026, 66.709853, -8.392223)
    RE.viewpoint().update()


    this.create('Button', 'rotate light', 'rotate light', 1)
    this.addText(
        'A point on the right arm is constrained to a ray.\n'
        'Shift-drag the ray origin to move it.\n'
        'Use ray yaw to rotate the ray in the XZ plane.\n'
        'You can also control the left arm orientation.\n'
    )
    this.create('Value_Slider', 'arm ori y', 'arm ori y', 1)
    this.findWidget('arm ori y').sliderRange(-math.pi, 0)
    this.findWidget('arm ori y').sliderValue(-math.pi / 2)

    this.create('Value_Slider', 'ray yaw', 'ray yaw', 1)
    this.findWidget('ray yaw').sliderRange(-math.pi, math.pi)
    this.findWidget('ray yaw').sliderValue(math.pi / 2)

    this.addText(
        '\nShift-drag to adjust the two foot targets and the ray origin.\n'
        'Green dots: ray. Red dot: constrained point.\n'
    )
    this.updateLayout()

    mMot = loadMotion(config.skel, config.motion)
    mLoader = mMot.loader

    for i in range(1, mLoader.numBone()):
        if mLoader.VRMLbone(i).numChannels() == 0:
            mLoader.removeAllRedundantBones()
            break
    mLoader._initDOFinfo()

    mMotionDOFcontainer = mMot.motionDOFcontainer
    mMotionDOF = mMotionDOFcontainer.mot

    # Motion/IK is in meters; renderer draws in centimeters.
    dofmat = mMotionDOF.matView()
    for i in range(mMotionDOF.rows()):
        dofmat.set(i, 1, dofmat(i, 1) + config.initialHeight)

    mSkin = RE.createVRMLskin(mLoader, True)
    mSkin.setMaterial('lightgrey_transparent')
    s = config.skinScale
    mSkin.scale(s, s, s)
    mSkin.setThickness(3 / s)

    mPose = m.vectorn()
    mPose.assign(mMotionDOF.row(0))

    # Create an initial pose with both arms extended forward.
    mLoader.setPoseDOF(mPose)
    left_shoulder = mLoader.getBoneByName('LeftShoulder')
    right_shoulder = mLoader.getBoneByName('RightShoulder')
    fk = mLoader.fkSolver()
    fk.localFrame(left_shoulder.treeIndex()).rotation.leftMult(
        m.quater(math.radians(-90), m.vector3(0, 1, 0)))
    fk.localFrame(right_shoulder.treeIndex()).rotation.leftMult(
        m.quater(math.radians(90), m.vector3(0, 1, 0)))
    fk.forwardKinematics()
    fk.getPoseDOFfromGlobal(mPose)
    mMotionDOF.row(0).assign(mPose)
    mSkin.setPoseDOF(mPose)

    mSolverInfo = createIKsolver(mLoader, config.IKconfig)
    effectors = mSolverInfo.effectors
    numCon = mSolverInfo.numCon

    mLoader.setPoseDOF(mPose)
    originalPos = lua.dynamic_list()
    for i in range(numCon):
        pos = effectors(i).bone.getFrame().toGlobalPos(effectors(i).localpos)
        originalPos[i] = pos * config.skinScale

    # This extra draggable point (index 2) controls the ray origin.
    originalPos.append(
        mLoader.bone(1).getFrame().translation * config.skinScale)
    mCON = RE.Constraints(originalPos)
    mCON.connect(__name__, 'eventFunction')

    limbik()


def limbik():
    global mPose, mLoader, mSkin, mMotionDOF, mSolverInfo, mCON

    this = m.getPythonWin()
    effectors = mSolverInfo.effectors
    numCon = mSolverInfo.numCon
    ik = mSolverInfo.solver

    mPose.assign(mMotionDOF.row(0))
    mLoader.setPoseDOF(mPose)

    hasRay = 1
    hasRot = 1

    ik._changeNumEffectors(numCon)
    ik._changeNumConstraints(hasRay + hasRot)

    footPos = m.vector3N(numCon)
    for i in range(numCon):
        ik._setEffector(i, effectors(i).bone, effectors(i).localpos)
        footPos(i).assign(mCON.conPos(i) / config.skinScale)

    # World-space ray: origin controlled by the third draggable point;
    # direction controlled by a yaw slider, with unit length by construction.
    rayOrigin = mCON.conPos(2) / config.skinScale
    yaw = this.findWidget('ray yaw').sliderValue()
    rayDir = m.vector3(math.cos(yaw), 0, math.sin(yaw))

    # Local point on the right arm that should lie on the ray.
    bone1 = mLoader.getBoneByName('RightElbow')
    lpos1 = m.vector3(-0.30, 0, 0)

    if hasRay:
        ik._setRayConstraint(0, bone1, lpos1, rayOrigin, rayDir)

    if hasRot:
        bone = mLoader.getBoneByName('LeftElbow')
        target_rotation = m.quater(
            this.findWidget('arm ori y').sliderValue(), m.vector3(0, 1, 0))
        ik._setOrientationConstraint(hasRay, bone, target_rotation)

    ik._effectorUpdated()
    ik.IKsolve(mPose, footPos)
    mSkin.setPoseDOF(mPose)
    mLoader.setPoseDOF(mPose)

    # Show a 1.5m ray as small green spheres (no extra drawing API required).
    RE.draw('Sphere', rayOrigin * config.skinScale,
            'marker_ray_origin', 'green_transparent', 5)

    points=m.vector3N()
    for j in range(1, 16):
        points.pushBack(rayOrigin + rayDir * (0.10 * j))

    RE.drawSpheres('ray_points', points * config.skinScale, 1.2, m.vector3(0,1,0))



def onCallback(w, userData):
    if w.id() == 'button1':
        print('button1')
    elif w.id() == 'rotate light':
        osm = RE.ogreSceneManager()
        if osm.hasSceneNode('LightNode'):
            node = osm.getSceneNode('LightNode')
            node.rotate(m.quater(math.radians(30), m.vector3(0, 1, 0)))
    limbik()


def dtor():
    pass


def frameMove(fElapsedTime):
    pass


def handleRendererEvent(ev, button, x, y):
    if mCON is not None:
        return mCON.handleRendererEvent(ev, button, x, y)
    return 0


def main():
    RE.createMainWin(sys.argv)
    ctor(m.getPythonWin())
    print('ctor finished')
    m.startMainLoop()


if __name__ == '__main__':
    main()
