// NodeStack.cpp: implementation of the NodeStack class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "nodestack.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NodeStack::NodeStack()
{
	top = -1;
}

void NodeStack::Initiate()
{
	top = -1;
}

NodeStack::~NodeStack()
{
}

void NodeStack::Push(Node* src)
{
	top++;
	ASSERT(top<NODE_STACK_SIZE);
	stack[top]= src;
}

void NodeStack::Pop(Node** psrc)
{
	if(top==-1) *psrc=NULL;
	else
	{
		*psrc=stack[top];
		top--;
	}
}

void NodeStack::GetTopNth(Node** psrc, int n)
{
	ASSERT(n>=0 && top-n>=0);
	*psrc=stack[top-n];
}