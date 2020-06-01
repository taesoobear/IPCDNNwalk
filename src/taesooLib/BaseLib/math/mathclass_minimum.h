#ifndef _MATHCLASS_MINIMUM_H_
#define _MATHCLASS_MINIMUM_H_

#include "../utility/checkError.h"
#include <assert.h>
#include <stdio.h>
#include <stdexcept>

#ifndef ASSERT
#ifdef _DEBUG

#define ASSERT(x) assert(x)
#define RANGE_ASSERT(x) assert(x)
#define VERIFY(x) assert(x)
#define TRACE	Msg::print
#else
#define ASSERT(x) 
#define RANGE_ASSERT(x) do {if(!(x)) throw std::runtime_error("range_error");} while(false)
//#define RANGE_ASSERT(x) assert(x)
//#define RANGE_ASSERT(x) 
#define VERIFY(x)	(x)
#define TRACE	__noop
#endif
#endif


#include "../utility/tfile.h"
#include <limits>

#include "math_macro.h"
#include "quater.h"
#include "vector3.h"
#include "traits.h"
#include "vectorn.h"
#include "matrixn.h"
#endif

