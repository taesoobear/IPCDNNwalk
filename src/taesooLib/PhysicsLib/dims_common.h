/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 */
/*!
 * @file   common.h
 * @author Katsu Yamane
 * @date   06/18/2003
 * @brief  Defines convenient macros used throughout the project.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

static char charname_separator = ':';

#ifndef PI
#define PI 3.1416
#endif

#define TINY 1e-8

#ifndef MAX
#define MAX(m, n) ((m >= n) ? m : n)
#define MIN(m, n) ((m <= n) ? m : n)
#endif
#ifndef MAX3
#define MAX3(l, m, n) ((MAX(l,m) >= n) ? MAX(l,m) : n)
#endif
#endif
