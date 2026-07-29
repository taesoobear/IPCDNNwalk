from libcalab_ogre3d import m, RE, lua, control # pip install libcalab-ogre3d (0.2.8 or later)
import numpy as np
import pdb, math
import libcalab_ogre3d

# please download the latest versions:
# pip install libcalab-ogre3d
# also, if ./work exists,
# cd work; git pull origin master

if libcalab_ogre3d.__version__<'0.3.1':
    print("Error: libcalab_ogre3d >= 0.3.1 is required. Please install the latest version.")
    os._exit(0)

def onCallback(w, userData):
    if w.id()=='erase all':
        RE.removeEntityByName('pnode')
        RE.removeEntityByName('pnode2')
        RE.removeEntityByName('pnode3')


this=RE.createMainWin()
this.addButton('erase all')
this.updateLayout()
scene_manager=RE.ogreSceneManager()


# particle count
N = 1000

xyz = np.zeros((N, 3), dtype=np.float32)
color = np.zeros((N, 4), dtype=np.uint8)
cov_diag = np.zeros((N, 3), dtype=np.float32)
cov_off = np.zeros((N, 3), dtype=np.float32)

idx = 0

# set 1
for y in range(10):
    for x in range(10):
        for z in range(10):
            # 평면 위에 점 배치 (z=0)
            xyz[idx] = [x*10, y*10, z*10]

            w=m.map(x,0,10, 0,1)
            wy=m.map(y,0,10, 0,1)
            wz=m.map(z,0,10, 0,1)
            color[idx] = [255*w, 255*wy, 255*wz, 255]   # red to green to black

            width=2.0
            cov_diag[idx] = [width*(w+0.1), width*(wy+0.1), width*(wz+0.1)]

            # off-diagonal = 0
            cov_off[idx] = [0.0, 0.0, 0.0]

            idx += 1


entity=m.createPointCloudEntity('particles_mesh', xyz.flatten(), color.flatten(), cov_diag.flatten(), cov_off.flatten(), N)
node=RE.ogreRootSceneNode().createChildSceneNode('pnode')
node.attachObject(entity)

if True:
    # spheres
    radius=np.ones((N), dtype=np.float32)*2.0
    entity3=m.createPointCloudEntity('particles3_mesh', xyz.flatten(), radius, color.flatten(), N)
    node3=RE.ogreRootSceneNode().createChildSceneNode('pnode3')
    node3.attachObject(entity3)
    node3.setPosition(m.vector3(100,0,0))

if True:
    n=1000
    noise=0.01
    # set 2
    # 단위구 위에 균일 분포 샘플
    theta = np.random.uniform(0, np.pi, n)
    phi   = np.random.uniform(0, 2 * np.pi, n)
    pts   = np.stack([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta),
    ], axis=1) + np.random.randn(n, 3) * noise

    entity2=m.createPointCloudEntity('particles2_mesh', (pts*50).astype(np.float32).flatten(), color.flatten(), cov_diag.flatten(), cov_off.flatten(), N)
    node2=RE.ogreRootSceneNode().createChildSceneNode('pnode2')
    node2.attachObject(entity2)
    node2.setPosition(m.vector3(-50,50,0))

while True:
    if not RE.renderOneFrame(True): break

