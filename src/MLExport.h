#ifndef _MLEXPORT_H_
#define _MLEXPORT_H_

#ifdef _MSC_VER
//#  if defined(ML_DLL) && (defined(_MSC_VER) || defined(__MWERKS__) || defined(__BORLANDC__) || __GNUC__ >= 3)
#    ifdef ML_LIBRARY
#      define ML_EXPORT	__declspec(dllexport)
#    else
#      define ML_EXPORT	__declspec(dllimport)
#    endif 
#else
#define ML_EXPORT
#endif

#endif
