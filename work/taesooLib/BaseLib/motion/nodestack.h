// NodeStack.h: interface for the NodeStack class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_)
#define AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define NODE_STACK_SIZE 100
class Node;

class NodeStack  
{
public:
	NodeStack();
	virtual ~NodeStack();
	void Initiate();

	Node* GetTop()		{ if(top<0) return NULL; return(stack[top]);};

	void Push(Node* src);
	void Pop(Node** psrc);
	void GetTopNth(Node** psrc, int n);	//!< n==0일때 top return
private:
	Node* stack[NODE_STACK_SIZE];
	int top;
};

#endif // !defined(AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_)
