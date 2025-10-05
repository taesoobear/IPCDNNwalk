// ModelLoader.cpp: implementation of the ModelLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ModelLoader.h"
#include "../BaseLib/utility/util.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NodeStack ModelLoader::m_TreeStack; //!< multi-threading시 주의 , 메모리 절약을 위해서 static으로 선언

ModelLoader::ModelLoader()
{
	m_pTreeRoot=NULL;
	m_nNumTreeNode=0;
}

ModelLoader::~ModelLoader()
{
	if(m_pTreeRoot) delete m_pTreeRoot;
}

int ModelLoader::GetIndex(Node* target) const
{
	ASSERT(target);
	ASSERT(m_apNode[target->GetIndex()]==target);
	return target->GetIndex();
}

int ModelLoader::GetIndex(const char *szNameID) const
{
	for(int j=0;j<m_nNumTreeNode ; j++)
	{
		if(m_apNode[j]->NameId && fast_strcmp_upper(m_apNode[j]->NameId,szNameID)==0) return j;
	}
	
	return -1;
}

Node* ModelLoader::GetNode(int nIndex) const	
{
	if(nIndex<0)
		throw std::runtime_error("nIndex<0 in ModelLoader::GetNode(nIndex)");
	return m_apNode[nIndex]; 
}


void ModelLoader::ExportNames(const char* filename)
{
	
	FILE* file;
	VERIFY(file=fopen(filename,"w"));

	for(int j=0;j<m_nNumTreeNode; j++)
	{
		fprintf(file,"%d %s\n", j, m_apNode[j]->NameId);
	}
	fclose(file);
}

int ModelLoader::CountTreeNode()
{
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;
			// do something for src
			// src의 index는 현재 index이다.
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}

	return index+1;
}


void ModelLoader::UpdateIndexes()
{
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;

			// update index
			m_apNode[index]=src;
			src->m_nIndex=index;
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
}

/*
Tree를 트래버스 하는 skeleton code입니다. (3가지 방법을 나열)

preorder search입니다.
void ModelLoader::Traverse()
{
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;
			// do something for src
			// src의 index는 현재 index이다.
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
}

// recursion version (index가 필요 없는 경우 이게 편할수도..)
void ModelLoader::printTree(Node* src)
{
	if( src != NULL )
	{
		TRACE("%s\n",src->NameId);
		printTree( src->m_pChildHead);		
		printTree( src->m_pSibling);
	}
}

// linear version
void ModelLoader::printTree()
{
	for(int i=0; i<GetNumTreeNode(); i++)
		TRACE("%s\n", GetNode(i)->NameId);
}
*/
