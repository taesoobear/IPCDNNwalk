markerFile="NONE"
local filename="../../taesoo_qp/Samples/sample_luatorch/models/cassie_final.wrl"
local loader=MainLib.VRMLloader(filename)
_createEmptyMotion(loader, 'empty.dof')
Start(filename, 'empty.dof' , 
0.0 , -- initial height
100  -- skin scale
)

