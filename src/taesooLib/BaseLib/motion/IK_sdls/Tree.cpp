#include <iostream>
using namespace std;

#ifdef WIN32
#include <windows.h>
#endif

#include "LinearR3.h"
#include "Tree.h"
#include "Node.h"

using namespace IK_sdls;

Tree::Tree()
{
	root = 0;
	nNode = nEffector = nJoint = 0;
}

void Tree::SetSeqNum(Node* node)
{
	switch (node->purpose) {
	case HINGEJOINT:
	case SLIDEJOINT:
		node->seqNumJoint = nJoint++;
		node->seqNumEffector = -1;
		break;
	case BALLJOINT:
		node->seqNumJoint = nJoint;
		nJoint+=3;
		node->seqNumEffector = -1;
		break;
	case EFFECTOR:
		node->seqNumJoint = -1;
		node->seqNumEffector = nEffector++;
		break;
	case FREEJOINT:
		node->seqNumJoint = nJoint;
		nJoint=nJoint+6;
		node->seqNumEffector = -1;
		break;
	}
}

void Tree::InsertRoot(Node* root)
{
	assert( nNode==0 );
	nNode++;
	Tree::root = root;
	assert( !(root->left || root->right) );
	SetSeqNum(root);
}

void Tree::InsertLeftChild(Node* parent, Node* child)
{
	assert(parent);
	assert(!parent->left);
	nNode++;
	parent->left = child;
	child->realparent = parent;

	assert( !(child->left || child->right) );
	assert(!child->IsEffector());
	SetSeqNum(child);
}

void Tree::InsertRightSibling(Node* parent, Node* child)
{
	assert(parent);
	assert(!parent->right);
	nNode++;
	parent->right = child;
	child->realparent = parent->realparent;
	assert( !(child->left || child->right) );
	assert(!child->IsEffector());
	SetSeqNum(child);
}
void Tree::InsertEffector(IK_sdls::Node* parent, IK_sdls::Effector* child)
{
	assert(parent);
	nNode++;
	child->realparent=parent;//->realparent;
	assert( !(child->left || child->right) );

	if(child->purpose!=DUMMY_EFFECTOR)
	{
		SetSeqNum(child);
		effectors.push_back(child);
	}
}

void Tree::InsertRelativeConstraint(IK_sdls::Node* parent1, IK_sdls::Node* parent2, IK_sdls::RelativeConstraint* child)
{
	InsertEffector(parent1, child);
	assert(child->_additionalNode->purpose==DUMMY_EFFECTOR);
	InsertEffector(parent2, child->_additionalNode);
}


void Tree::RemoveAllEffectors()
{
	for(int i=0; i<effectors.size(); i++)
		delete effectors[i];
	effectors.clear();
	nNode=nJoint;
	nEffector=0;
}

void Tree::InsertChild_automatic(IK_sdls::Node* parent, IK_sdls::Node* child)
{
	if(parent->left)
	{
		parent=parent->left;
		while(parent->right)
			parent=parent->right;

		InsertRightSibling(parent, child);
	}
	else
	{
		InsertLeftChild(parent, child);
	}
}
// Search recursively below "node" for the node with index value.
Node* Tree::SearchJoint(Node* node, int index)
{
	Node* ret;
	if (node != 0) {
		if (node->seqNumJoint == index) {
			return node;
		} else {
			if (ret = SearchJoint(node->left, index)) {
				return ret;
			}
			if (ret = SearchJoint(node->right, index)) {
				return ret;
			}
			return NULL;
		}
	} 
	else {
		return NULL;
	}
}


// Get the joint with the index value
Node* Tree::GetJoint(int index)
{
	return SearchJoint(root, index);
}

/*
// Search recursively below node for the end effector with the index value
Node* Tree::SearchEffector(Node* node, int index)
{
	Node* ret;
	if (node != 0) {
		if (node->seqNumEffector == index) {
			return node;
		} else {
			if (ret = SearchEffector(node->left, index)) {
				return ret;
			}
			if (ret = SearchEffector(node->right, index)) {
				return ret;
			}
			return NULL;
		}
	} else {
		return NULL;
	}
}
*/


// Get the end effector for the index value
Node* Tree::GetEffector(int index)
{
	assert(effectors[index]->seqNumEffector==index);
	return effectors[index];
}

// Returns the global position of the effector.
const vector3& Tree::GetEffectorPosition(int index)
{
	Node* effector = GetEffector(index);
	assert(effector);
	return (effector->_global.translation);  
}

#include <stdio.h>
void Tree::ComputeTree(Node* node)
{
	if (node != 0) {
		//printf("node %d\n", node->purpose);
		node->ComputeS();
		node->ComputeW();
		ComputeTree(node->left);
		ComputeTree(node->right);
	}

}
void Tree::ComputeDS(Node* node)
{
	if (node != 0) {
		//printf("node %d\n", node->purpose);
		node->ComputeDS();
		ComputeDS(node->left);
		ComputeDS(node->right);
	}
}
void Tree::ComputeDQfromDS(Node* node)
{
	if (node != 0) {
		//printf("node %d\n", node->purpose);
		node->ComputeDQfromDS();
		ComputeDQfromDS(node->left);
		ComputeDQfromDS(node->right);
	}
}

void Tree::Compute(void)
{ 
	ComputeTree(root); 

	Node* n;
	for(int ii=0; ii<effectors.size(); ii++)
	{
		n = effectors[ii];
		n->ComputeS();
	}
}


void Tree::PrintTree(int depth,Node* node)
{
	if (node != 0) {
		node->PrintNode(depth);
		PrintTree(depth+1,node->left);
		PrintTree(depth, node->right);
	}
}

void Tree::Print(void) 
{ 
	PrintTree(0, root);  
	cout << "\n";
}

// Recursively initialize tree below the node
void Tree::InitTree(Node* node)
{
	if (node != 0) {
		node->InitNode();
		InitTree(node->left);
		InitTree(node->right);
	}
}

// Initialize all nodes in the tree
void Tree::Init(void)
{
	InitTree(root);
}

void Tree::UnFreezeTree(Node* node)
{
	if (node != 0) {
		node->UnFreeze();
		UnFreezeTree(node->left);
		UnFreezeTree(node->right);
	}
}

void Tree::UnFreeze(void)
{
	UnFreezeTree(root);
}
