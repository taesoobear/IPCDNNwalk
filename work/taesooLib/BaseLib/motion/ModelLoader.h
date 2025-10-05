#ifndef MODELLOADER_H_
#define MODELLOADER_H_

#pragma once

#include "node.h"
#include "nodestack.h"
#include <list>
class PLDPrim;

//! Abstract class for model hierarchy, animation loading
class ModelLoader  : noncopyable
{
public:
	ModelLoader();
	virtual ~ModelLoader();

	//! model hierarchy tree의 root
	Node* m_pTreeRoot;

	inline int GetNumTreeNode() const				{ return m_nNumTreeNode;};
	int GetIndex(Node* target) const;
	int GetIndex(const char *szNameID) const;
	char* GetName(int index)						{ return m_apNode[index]->NameId;};

	//! 부모노드의 첫번째 child node를 return한다. 없으면 NULL return
	inline Node* GetFirstChild(Node* pParent) const		{ return pParent->m_pChildHead; };
	//! 이전 child node를 입력받아 다음 child node를 return한다.
	inline Node* GetNextSibling(Node* pPrevChild) const	{ return pPrevChild->m_pSibling; };
	//! Traverse순서가 nIndex번째에 해당하는 node를 return 한다.
	Node* GetNode(int nIndex) const	;

	void ExportNames(const char* filename);

	// tree 를 traversal하면서, index와 parent를 세팅한다.
	void UpdateIndexes();

	static NodeStack m_TreeStack; //!< multi-threading시 주의 , 메모리 절약을 위해서 static으로 선언
	

protected:
	int CountTreeNode();
	//! 모든 Node pointer들이 tree inorder traversal 순으로 저장된다.
	/*!
	생성자에서 초기화 되도록 구현한다.
	파일에 노드들이 저장되어 있는 순서이다. 로딩할때 앞에서부터 채워진다.
	로딩이 끝난후 필요 없지만, VertexBuffer를 만들때 tree traversal을 생략하기 위해서
	남겨놓았다. 또한 modeling tool을 만들때도 사용할 수 있다.*/
	std::vector<Node*> m_apNode;

	int m_nNumTreeNode;		//!< m_nNumTreeNode==m_apNode.size();
};

#endif
