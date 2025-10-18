import numpy as np
import torch

import genesis as gs

if int(gs.__version__[2])==2:
    from .scene_v2 import Scene 
else:
    from .scene_v3 import Scene 
