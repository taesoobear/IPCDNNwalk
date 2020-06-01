// stdafx.cpp : 표준 포함 파일만 들어 있는 소스 파일입니다.
// OgreFltk.pch는 미리 컴파일된 헤더가 됩니다.
// stdafx.obj에는 미리 컴파일된 형식 정보가 포함됩니다.

#include "stdafx.h"
void dprintf(bool bExpression,const char* pszFormat, ...){
	if(bExpression){
		va_list argList;
		va_start(argList,pszFormat);
		vprintf(pszFormat, argList);
		fflush(stdout);
	}
}

// TODO: 필요한 추가 헤더는
// 이 파일이 아닌 stdafx.h에서 참조합니다.
