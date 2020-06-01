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
#ifndef OPENHRP_CONFIG_H_INCLUDED
#define OPENHRP_CONFIG_H_INCLUDED

// for Windows DLL export 
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# ifdef HRPMODEL_MAKE_DLL
#   define HRPMODEL_EXPORT __declspec(dllexport)
# else 
#   define HRPMODEL_EXPORT __declspec(dllimport)
# endif
#else 
# define HRPMODEL_EXPORT 
#endif /* Windows */


namespace OpenHRP
{
	static const double GRAVITY_ACCELERATION = 9.80665;
};


#endif
