#ifndef _TUGL_HPP_
#define _TUGL_HPP_

#pragma once
#include "tugl_impl.h"
namespace TUGL
{
	// undirected graph
struct EmptyEdge
{
};

template <class NodeType, class EdgeType>
class edge;

template <class NodeType, class EdgeType=EmptyEdge>
class node
{
public:
	// always reference
	node_struct* _ptr;

	node(): _ptr(NULL){}
	node(node_struct* p):_ptr(p){}

	int index()	const		{ return _ptr->_index;}
	NodeType& data() const			{ return *((NodeType*)_ptr->_data);}
	NodeType* operator->() const	{ return &data();}

	TUGL::edge<NodeType, EdgeType> edge(int index) const;

	edge_struct* getEdgePtr(int index) const		{ return _edge()[index];}

	int  degree()    const	{ return _edge().size(); }

	inline operator node_struct*()					{ return _ptr;}
	friend bool operator==(node const& a, node const& b)
	{
		return a._ptr==b._ptr;
	}

	// almost private:
	const std::vector<edge_struct*>& _edge() const	{ return _ptr->_aE;}
};

template <class NodeType, class EdgeType=EmptyEdge>
class edge
{
public:
	edge():_ptr(NULL){}
	edge(edge_struct* ptr):_ptr(ptr){}
	edge_struct* _ptr;

	node<NodeType, EdgeType> v1()    const { return _ptr->_s;}
	node<NodeType, EdgeType> v2()    const { return _ptr->_t;}
	node<NodeType, EdgeType> target(node<NodeType, EdgeType> v) {return node<NodeType, EdgeType>(_ptr->target(v._ptr));	}
	int index() const		{ return _ptr->_index;}

	EdgeType& data() const			{ return *((EdgeType*)_ptr->_data);}
	EdgeType* operator->() const	{ return &data();}

	inline operator edge_struct*()					{ return _ptr;}
	friend bool operator==(edge const& a, edge const& b)
	{
		return a._ptr==b._ptr;
	}
};

template <class NodeType, class EdgeType>
edge<NodeType, EdgeType> node<NodeType, EdgeType>::edge(int index) const
{
	TUGL::edge<NodeType, EdgeType> e;
	e._ptr=getEdgePtr(index);
	return e;
}

/** 가정: 에지나 node가 추가로 생길수는 있지만 중간에 지워지지는 않는다. 그래프 자료구조에서는 상관없지만, node_array나 edge_array의 구현에서 빠른 indexing 속도와 쉬운 구현을 위해 array를 사용했기 때문. 나중에 이 가정을 없앨 필요가 있으면 아마도 interface는 유지한채로 재구현이 가능할 것이다.
*/
template <class NodeType, class EdgeType=EmptyEdge>
class graph {

	graph_impl mG;
public:
	graph()					{}
	virtual ~graph()		{ clear(); }
	virtual void clear()
	{
		for(int i=0; i<mG.m_aV.size(); i++)
		{
			delete (NodeType*)mG.m_aV[i]->_data;
		}
		for(int i=0; i<mG.m_aE.size();i++)
		{
			delete (EdgeType*)mG.m_aE[i]->_data;
		}

		mG.clear();
	}

	int  numNodes() const   { return mG.m_aV.size(); }
	int  numEdges() const   { return mG.m_aE.size(); }

	// if you know the number of nodes to be created, please reserve NodeArray for efficiency
	void reserveNodeArray(int numNodes)		{ mG.m_aV.reserve(numNodes);}
	void reserveEdgeArray(int numEdges)		{ mG.m_aE.reserve(numEdges); }

	/// 새 노드생성과 edge생성은 반드시 newNode, newEdge를 사용할것.
	node<NodeType, EdgeType> newNode()
	{ return node<NodeType, EdgeType>(mG.newNode(new NodeType()));}

	edge<NodeType, EdgeType> newEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w)
	{ return edge<NodeType, EdgeType>(mG.newEdge(v._ptr, w._ptr,new EdgeType()));}

    edge<NodeType, EdgeType> findEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w)
	{
		return edge<NodeType, EdgeType>(mG.findEdge(v._ptr, w._ptr));
	}
	edge<NodeType, EdgeType> findEdge(int index) const { return edge<NodeType, EdgeType>(mG.edge(index));}
	node<NodeType, EdgeType> findNode(int index) const { return node<NodeType, EdgeType>(mG.node(index));}

	inline node_struct* getNodePtr(int index) const	{ return mG.node(index);}
	inline edge_struct* getEdgePtr(int index) const	{ return mG.edge(index);}
};

}// end namespace

#define TUGL_for_all_node TUGL_for_all_nodes
#define TUGL_forall_nodes TUGL_for_all_nodes
#define TUGL_for_all_edge TUGL_for_all_edges

#define TUGL_for_all_nodes(v,g)\
	for(int _tugl_i=0; _tugl_i<(g).numNodes() &&((v)._ptr=(g).getNodePtr(_tugl_i))!=NULL; _tugl_i++)

#define TUGL_for_all_edges(e,g)\
	for(int _tugl_i=0; _tugl_i<(g).numEdges() && ((e)._ptr=(g).getEdgePtr(_tugl_i))!=NULL; _tugl_i++)

#define TUGL_for_adj_edges(e,v)\
	for(int _tugl_i=0; _tugl_i<(v).degree() && ((e)._ptr=(v).getEdgePtr(_tugl_i))!=NULL; _tugl_i++)

/*
forall_adj_nodes(v, w) { ... }
iterates over all nodes v that are adjacent to the node w. In the case of a directed graph, these are all v for which there is an edge (w,v) in the graph. In the undirected case, these are all v for which there is an edge (w,v) or an edge (v,w). */

#define TUGL_forall_adj_nodes(v,w)\
	for(int _tugl_i=0; _tugl_i<(w).outdeg() && ((v)._ptr=(w).getOutEdgePtr(0)->_t)!=NULL; _tugl_i++)



#endif
