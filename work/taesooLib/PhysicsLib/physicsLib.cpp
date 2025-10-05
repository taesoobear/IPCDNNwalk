
#include "physicsLib.h"
void dprintf(bool bExpression,const char* pszFormat, ...){
	if(bExpression){
		va_list argList;
		va_start(argList,pszFormat);
		vprintf(pszFormat, argList);
		fflush(stdout);
	}
}

