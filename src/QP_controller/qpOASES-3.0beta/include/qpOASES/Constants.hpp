/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qpOASES/Constants.hpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2007-2012
 *
 *	Definition of all global constants.
 */


#ifndef QPOASES_CONSTANTS_HPP
#define QPOASES_CONSTANTS_HPP


#include <qpOASES/Types.hpp>


BEGIN_NAMESPACE_QPOASES

#ifdef EPS
#undef EPS
#endif

/** Numerical value of machine precision (min eps, s.t. 1+eps > 1).
	Note: this value has to be positive! */
#ifdef __USE_SINGLE_PRECISION__
const real_t EPS = 1.193e-07;
#else
const real_t EPS = 2.221e-16;
#endif /* __USE_SINGLE_PRECISION__ */


/** Numerical value of zero (for situations in which it would be
 *	unreasonable to compare with 0.0).
 *  Note: this value has to be positive! */
const real_t ZERO = 1.0e-50;

/** Numerical value of infinity (e.g. for non-existing bounds).
	Note: this value has to be positive! */
const real_t INFTY = 1.0e12;



END_NAMESPACE_QPOASES


#endif	/* QPOASES_CONSTANTS_HPP */


/*
 *	end of file
 */
