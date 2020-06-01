// Node.h: interface for the Node class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
#define AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "my.h"
#include "../utility/tfile.h"
#include "../utility/TextFile.h"
#include <list>
/* TreeNode
*/
class Node  
{
public:
	Node();
	virtual ~Node();
	
	virtual void pack(BinaryFile& file, int nVersion) const;
	virtual void unpack(BinaryFile& file, int nVersion) ;

	void AddChild(Node* pChild);
	void RemoveChild(Node* pChild);
	void updateChildTail();

	void detachAllChildren(std::list<Node*>& children);
	void getAllChildren(std::list<Node*>& children);
	void addChildren(std::list<Node*>& children);
	int numChildren();

	void SetNameId(const char* name);
	const char* GetNameId() const	{return NameId;}
	int CountChildren();

	virtual void printHierarchy(int depth=0);

	char* NameId;	//!< Node의 이름을 갖는 경우 사용됨 
	Node *m_pChildHead;	//!< 자식노드 list의 head, singly linked list이다. 
	Node *m_pChildTail; //!< 자식노드 list의 tail
	Node *m_pSibling;	//!< 자식노드들끼리 연결하는 pointer, 오른쪽 sibling을 뜻한다. 
	Node *m_pParent;
	int m_nIndex;
	int GetIndex() const	{return m_nIndex;}

	// how to iterate parents:
	// 		for(Node* c=bone->m_pParent; c->m_pParent->m_pParent; c=c->m_pParent)
	// or using a type-cast
	// 		for(Bone* c=(Bone*)bone->m_pParent; c->m_pParent->m_pParent; c=(Bone*)(c->m_pParent))

	
	// how to iterate children:
	// 1)
	// 		for(Node* c=m_pChildHead; c; c=c->m_pSibling)
	// or using a type-cast
	// 		for(Bone* c=(Bone*)bone->m_pChildHead; c; c=(Bone*)(c->m_pSibling))
	// 2)
	// 		for(child_iterator i= begin(); i!=end(); ++i)

	struct child_iterator
	{
		Node* value;
		inline void operator++() 	{value=value->m_pSibling;	}
		inline Node* operator*()	{ return value;}
		inline bool operator==(child_iterator const& o) {return value==o.value;}
		inline bool operator!=(child_iterator const& o) {return value!=o.value;}
	};

	child_iterator begin();
	child_iterator end();

	// deprecated.
	NodeType_T NodeType;	//!< TransformNode인지 IndexedFaceSetNode인지 등을 구분한다.

};

#endif // !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
