// Node.cpp: implementation of the Node class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "node.h"

//#include "headers.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Node::Node()
{
	m_pChildHead=NULL;
	m_pChildTail=NULL;
	m_pSibling=NULL;
	m_pParent=NULL;
	m_nIndex=-1;
	NodeType=GENERAL;
	NameId=NULL;
}

Node::~Node()
{
	if(m_pChildHead)
		delete m_pChildHead;
	if(m_pSibling)
		delete m_pSibling;
	if(NameId) delete[] NameId;
}

void Node::pack(BinaryFile& file, int nVersion) const
{
	file.packInt(NodeType);
	file.pack(NameId);
}

void Node::unpack(BinaryFile& file, int nVersion) 
{
	// NodeType을 unpack하는 과정이 없음에 주의할것 (밖에서 NodeType을 unpack해야 type에 따라서 new를 해야하기 때문)
	TString name;
	file.unpack(name);

	if(name.length())
	{
		NameId=new char[name.length()+1];
		strcpy(NameId, name.ptr());
	}
}

void Node::AddChild(Node* pChild)
{
	if(!m_pChildHead)
	{
		m_pChildHead=pChild;
		m_pChildTail=pChild;
	}
	else
	{
		m_pChildTail->m_pSibling=pChild;
		m_pChildTail=pChild;
	}

	pChild->m_pParent=this;
}

void Node::RemoveChild(Node* pChild)
{
	Node* pc=m_pChildHead;

	for(Node* c=m_pChildHead; c!=NULL; c=c->m_pSibling)
	{
		if(c==pChild)
		{
			if(pc==pChild)
				m_pChildHead=c->m_pSibling;
			else
				pc->m_pSibling=c->m_pSibling;

			c->m_pSibling=NULL;
			delete c;
			break;
		}
		pc=c;
	}
	updateChildTail();
}
void Node::updateChildTail()
{
	m_pChildTail=m_pChildHead;

	if(m_pChildTail)
	{
		while(m_pChildTail->m_pSibling)
			m_pChildTail=m_pChildTail->m_pSibling;
	}
}

void Node::SetNameId(const char* name)
{
	if(NameId) delete[] NameId;
	if(!name) {
		NameId=NULL;
		return ;
	}
	NameId=new char[strlen(name)+1];
	strcpy(NameId, name);
}


Node::child_iterator Node::begin()
{
	child_iterator i;
	i.value=m_pChildHead;
	return i;
}

Node::child_iterator Node::end()
{
	child_iterator i;
	i.value=NULL;
	return i;
}

int Node::CountChildren()
{
	int count=0;
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		count++;
	return count;
}

void Node::detachAllChildren(std::list<Node*>& children)
{
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
	{
		children.push_back(i);
	}

	for(std::list<Node*>::iterator i=children.begin(); i!=children.end();++i)
	{
		(*i)->m_pParent=NULL;
		(*i)->m_pSibling=NULL;
	}

	m_pChildHead=NULL;
	m_pChildTail=NULL;
}

void Node::getAllChildren(std::list<Node*>& children)
{
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		children.push_back(i);
}

int Node::numChildren()
{
	std::list<Node*> children;
	getAllChildren(children);
	return children.size();
}
void Node::addChildren(std::list<Node*>& children)
{
	std::list<Node*>::iterator i;
	for(i=children.begin(); i!=children.end(); i++)
	{
		AddChild(*i);
	}
}

void Node::printHierarchy(int depth)
{
	for(int ii=0; ii<depth; ii++) printf(" ");
	Msg::print("%s \n", NameId);
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		i->printHierarchy(depth+1);
}
