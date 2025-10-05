// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
#ifndef MODEL_LOADER_UTIL_H_INCLUDED
#define MODEL_LOADER_UTIL_H_INCLUDED

#include <string>
#include <sstream>

//#include <ModelLoader.h>
//#include "ORBwrap.h"
#include "World_UT.h"
#include "hrpModelExportDef.h"
#include "../BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "sDIMS/chain.h"

class World;

namespace OpenHRP
{
	HRPMODEL_EXPORT int loadBodyFromCharacterInfo(World_UT_base* world, const char* name, const CharacterInfo* charaInfo);
	/*HRPMODEL_EXPORT int loadBodyFromModelLoader(World* world, const char* name,const char *url, CORBA_ORB_var orb);
	HRPMODEL_EXPORT int loadBodyFromModelLoader(World* world, const char* name, const char *url, CosNaming::NamingContext_var cxt);
	HRPMODEL_EXPORT int loadBodyFromModelLoader(World* world, const char* name, const char *url, int argc, char *argv[]);
	HRPMODEL_EXPORT int loadBodyFromModelLoader(World* world, const char* name, const char *url, std::istringstream& strm);*/
};

namespace MotionUtil
{


	// ???? ???ì¿??????????? ?À¸?.
	void FullbodyIK_UTPoser_setParam(MotionUtil::FullbodyIK_MotionDOF* solver, int numiter, double stepsize);
	FullbodyIK_MotionDOF* createFullbodyIk_UTPoser(MotionDOFinfo const& info, std::vector<Effector>& effectors);
}


#endif
