#ifndef _TUGL_IMPL_H_
#define _TUGL_IMPL_H_
#pragma once
#include <vector>
#include <list>
namespace TUGL
{

struct edge_struct;
struct node_struct : noncopyable
{
	node_struct();
	node_struct(int index);
	~node_struct();

	int _index;			// internal name (index)

	std::vector<edge_struct*> _aE;	//!< reference array로 초기화 함에 주의
	void* _data;
};

struct edge_struct : noncopyable
{
	edge_struct();
	~edge_struct();
	edge_struct(node_struct* v, node_struct* w, int index);

	node_struct* target(node_struct* v);

	int  _index;          // internal name (index)
	node_struct* _s;             // source node (a node with smaller index)
	node_struct* _t;             // target node
	void* _data;
};


struct graph_impl : noncopyable
{
	graph_impl(){}
	~graph_impl(){}
	std::vector<node_struct*> m_aV;              //!< list of all nodes
	std::vector<edge_struct*> m_aE;              //!< list of all edges	 (낮은 인덱스에서 높은 인덱스로 가는 에지)

	node_struct* newNode(void* data);
	edge_struct* newEdge(node_struct* s, node_struct* v, void* data);
	edge_struct* findEdge(node_struct* s, node_struct* v);
	node_struct* node(int i) const;
	edge_struct* edge(int i) const;
	void clear();
};


}// end of namespace TUGL
#endif
