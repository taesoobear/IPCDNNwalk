#include "gjoint_fixed.h"
#include "gjoint.h"
#include "liegroup.h"
#include "rmatrix3j.h"



//=============================================================
//                 GJointFixed
//=============================================================
GJointFixed::GJointFixed()
{
	jointType = GJOINT_FIXED;
	allocate_memory(0);
}

