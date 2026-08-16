
import torch, pdb
from mhr.mhr import MHR # pip install mhr pymomentum-cpu
import trimesh
import libcalab_ogre3d
from libcalab_ogre3d import RE, m, lua, control
import numpy as np

assert(libcalab_ogre3d.__version__>='0.3.3')

torch.manual_seed(0)

def onCallback(w, userdata):
    if w.id()=='draw overlapped':
        global skin1
        if w.checkButtonValue():
            skin1.setTranslation (0,0,0)
        else:
            skin1.setTranslation (-100,0,0)


def drawMesh(vertices, faces_new, name='mesh'):
    print('Vertices shape =', vertices.shape)

    tl_mesh=m.Mesh()

    tl_mesh.init(lua.vec(vertices.flatten()).vec3View(), lua.ivec(faces_new.flatten()))
    tl_mesh.calculateVertexNormal()
    meshToEntity, node=tl_mesh.drawMesh('lightgrey_transparent', name+'_node')
    return node

def _prepare_input_data(batch_size: int) -> torch.Tensor:
    identity_coeffs = 0.8 * torch.randn(batch_size, 45).cpu()
    model_parameters = 0.4 * (torch.rand(batch_size, 204) - 0.5).cpu()
    face_expr_coeffs = 0.3 * torch.randn(batch_size, 72).cpu()
    return identity_coeffs, model_parameters, face_expr_coeffs

def createLoaderFromMHR(asset_folder, identity_coeffs, face_expr_coeffs, lod=4):
    """
    Create a taesooLib FBX loader from a meta MHR model.

    Args:
        asset_folder (pathlib.Path):
            Path to the MHR asset folder. 

        identity_coeffs: 
            Identity coefficients used to define the character's body/shape.

        face_expr_coeffs:
            Facial expression coefficients.

        lod (int, optional):
            Level of detail (LOD) of the MHR model. Defaults to 4.

    Returns:
        loader:
            A taesooLib FBX loader initialized with the generated MHR character.
    """
    batch_size=1
    lod_level=lod
    temp_mhr_model = MHR.from_files(folder=asset_folder, device=torch.device("cpu"), lod=lod_level)
    model_parameters= torch.zeros(batch_size, 204).cpu()
    with torch.no_grad():
        verts, skel_state = temp_mhr_model(identity_coeffs.reshape(batch_size, 45), model_parameters, face_expr_coeffs.reshape(batch_size, 72))
    global_pos   = skel_state[..., 0:3]   # [B, J, 3]
    global_quat  = skel_state[..., 3:7]   # [B, J, 4]
    global_scale = skel_state[..., 7:8]   # [B, J, 1]


    fbx=RE.FBXloader(str(asset_folder / f'lod{lod_level}.fbx')) # loading  (incorrect but will be fixed below)

    skeleton = temp_mhr_model.character.skeleton
    for i, joint in enumerate(skeleton.joints):
        print(i, joint.name, fbx.loader.bone(i).name())
        assert(i==0 or (joint.name== fbx.loader.bone(i).name()))

    ibatch=0



    if True:
        # update shape
        subMesh=fbx.fbxInfo[1]
        mesh=subMesh.mesh
        # build vertex index map

        faces_b=temp_mhr_model.character.mesh.faces
        nface=mesh.numFace()
        assert (nface==faces_b.shape[0])

        vertex_map = np.full(mesh.numVertex(), -1, dtype=np.int32)

        for i in range(nface):
            for j in range(3):
                va=mesh.getFace(i).vertexIndex(j)
                vb=faces_b[i,j]
                if vertex_map[va] != -1:
                    assert vertex_map[va] == vb, \
                        f"Inconsistent mapping: {va} -> {vertex_map[va]} / {vb}"
                else:
                    vertex_map[va] = vb

        mesh.vertexBuffer().array[...]=verts[ibatch][vertex_map].numpy()

    # update bindpose and mesh
    for ibone in range(1, global_quat.shape[1]):
        fbx.loader.bone(ibone).getFrame().assign(m.transf(RE.V4toQuater(global_quat[ibatch, ibone]), RE.toVector3(global_pos[ibatch, ibone])))
    fbx.loader.fkSolver().inverseKinematicsExact() 
    #fbx.loader.fkSolver().inverseKinematics() 
    fbx._setBindPose(fbx.loader)
    fbx._bindPoseUpdated()
    return fbx


def createPoseFromMHR(fbx, skel_state):
    assert(len(skel_state.shape)==2)
    global_pos   = skel_state[..., 0:3]   # [B, J, 3]
    global_quat  = skel_state[..., 3:7]   # [B, J, 4]
    global_scale = skel_state[..., 7:8]   # [B, J, 1]

    for ibone in range(1, global_quat.shape[0]):
        fbx.loader.bone(ibone).getFrame().assign(m.transf(RE.V4toQuater(global_quat[ibone]), RE.toVector3(global_pos[ ibone])))

    fbx.loader.fkSolver().inverseKinematics()
    pose=fbx.loader.pose()
    return pose

def setSkinState(skin, skel_state):
    # If using pred_joint_coords from SAM 3D Body instead of skel_state,
    # compute global_pos as follows.
    #global_pos = pred_joint_coords * np.array([100.0, -100.0, -100.0], dtype=np.float32)
    fkSolver=skin.getState()
    assert(len(skel_state.shape)==2)
    global_pos   = skel_state[..., 0:3]   # [B, J, 3]
    global_quat  = skel_state[..., 3:7]   # [B, J, 4] == Quaternions computed from pred_joint_rots
    unused_global_scale = skel_state[..., 7:8]   # [B, J, 1]

    for ibone in range(1, global_quat.shape[0]):
        fkSolver.globalFrame(ibone).assign(m.transf(RE.V4toQuater(global_quat[ibone]), RE.toVector3(global_pos[ ibone])))

    fkSolver.inverseKinematicsExact()
    skin.setSamePose(fkSolver)


# main
this=RE.createMainWin()
this.addCheckButton('draw overlapped', False)
this.updateLayout()

# https://github.com/facebookresearch/MHR
asset_folder= RE.path("../MHR/assets")
lod_level=4


# create two random identities
batch_size = 2
identity_coeffs, model_parameters, face_expr_coeffs = _prepare_input_data(batch_size)


# loader for identity 0
fbx=createLoaderFromMHR(asset_folder, identity_coeffs[0], face_expr_coeffs[0], lod=lod_level)

skinScale=1
skin1=RE.createFBXskin(fbx, True)
skin1.setScale(skinScale, skinScale, skinScale)
skin1.setTranslation (-100,0,0)

if True:
    # create mesh snapshots for comparison
    mhr_model = MHR.from_files(folder=asset_folder, device=torch.device("cpu"), lod=lod_level)

    with torch.no_grad():
        verts, skel_state = mhr_model(identity_coeffs, model_parameters, face_expr_coeffs)

    # this works  too
    #skin1.setPose(createPoseFromMHR(fbx, skel_state[0]))

    # this is more accurate
    setSkinState(skin1, skel_state[0])


    mesh = trimesh.Trimesh(vertices=verts[0].numpy(), faces=mhr_model.character.mesh.faces, process=False)
    output_mesh_path = "./test.ply"
    mesh.export(output_mesh_path)
    print(f"Saved example MHR mesh to {output_mesh_path}")
    drawMesh(verts[0], mhr_model.character.mesh.faces)
    drawMesh(verts[1], mhr_model.character.mesh.faces, 'mesh2').translate(m.vector3(100,0,0))


m.startMainLoop() # this finishes when program finishes
