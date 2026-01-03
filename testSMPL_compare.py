import torch
import os,sys, pdb, math, random, copy 
from libcalab_ogre3d import RE, m, lua, control
import smplx # pip install smplx
import numpy as np

datasetRoot=RE.path('../../d/sample_SAMP') # RE.path returns pathlib.Path (normalized)
model_path=datasetRoot / 'SMPL-X/model1.1/models'
bm_path=model_path / 'smplx/SMPLX_NEUTRAL.npz'
assert(bm_path.is_file())

def onCallback(w, userdata):
    if w.id()=='overlap':
        global skin
        if w.checkButtonValue():
            skin.setTranslation(0,0,0)
        else:
            skin.setTranslation(100,0,0)

def drawSMPLmesh_using_original_implementation(model_folder, betas, sample_expression=False):
    model = smplx.create(model_folder, model_type='smplx',
                         gender='neutral', use_face_contour=False,
                         num_betas=10,
                         num_expression_coeffs=10,
                         ext='npz')
    print(model)

    expression = None
    if sample_expression:
        expression = torch.randn(
            [1, model.num_expression_coeffs], dtype=torch.float32)

    output = model(betas=betas, expression=expression,
                   return_verts=True)
    vertices = output.vertices.detach().cpu().numpy().squeeze()
    joints = output.joints.detach().cpu().numpy().squeeze()
    RE.drawBillboard(lua.vec(joints.flatten()).vec3View()*100,'joints','redCircle',5,'QuadListV')

    print('Vertices shape =', vertices.shape)
    print('Joints shape =', joints.shape)

    tl_mesh=m.Mesh()

    faces_new=model.faces.copy()
    tl_mesh.init(lua.vec(vertices.flatten()).vec3View(), lua.ivec(faces_new.flatten()))
    tl_mesh.calculateVertexNormal()
    meshToEntity, node=tl_mesh.drawMesh('lightgrey_transparent', 'mesh_node')
    node.scale(100,100,100)

# main
this=RE.createMainWin(sys.argv)


betas = torch.randn([1, 10], dtype=torch.float32)
drawSMPLmesh_using_original_implementation(str(model_path), betas)

fbx=RE.createSMPLskeleton(str(bm_path), betas.numpy().flatten())
skinScale=100 # rendering in cm unit.

drawSkeleton=True
skin=RE.createFBXskin(fbx, drawSkeleton)
skin.setScale(skinScale, skinScale, skinScale)
skin.setTranslation(100,0,0)
skin.setMaterial('red_transparent')

this.addCheckButton('overlap',False)
this.updateLayout()

while True:
    checkEvents=True
    if not RE.renderOneFrame(checkEvents): break

