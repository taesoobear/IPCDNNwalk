#include "stdafx.h"

#include <vector>
#include <list>
#include "tugl_impl.h"
struct edge_struct;

namespace TUGL
{
	node_struct::node_struct():_index(-1), _data(NULL){}
	node_struct::node_struct(int index):_data(NULL) { _index=index;}
	node_struct::~node_struct()		{}

	edge_struct::edge_struct():_index(-1),_s(NULL),_t(NULL),_data(NULL)	{}
	edge_struct::~edge_struct()						{}
	edge_struct::edge_struct(node_struct* v, node_struct* w, int index) {_index=index;_s=v;_t=w;}

	node_struct* edge_struct::target(node_struct* v)
	{
		if(v==_s) return _t;
		if(v==_t) return _s;
		return NULL;
	}		

	node_struct* graph_impl::newNode(void* data)
	{
		m_aV.push_back(new node_struct(m_aV.size()));
		m_aV.back()->_data=data;
		
		return m_aV.back();
	}

	void check_vertex(graph_impl const& g, int ii)
	{
		node_struct* v=g.m_aV[ii];
		ASSERT(v->_index==ii);
//		for(int i=0; i<v->_aE.size(); i++)
//			ASSERT(g.m_aE[v->_aE[i]->_index]==v->_aE[i]);

	}
	

	edge_struct* graph_impl::newEdge(node_struct* v, node_struct* w, void* data)
	{
	#ifdef _DEBUG
		check_vertex(*this, v->_index);
		check_vertex(*this, w->_index);
		ASSERT(v!=w);
		ASSERT(!findEdge(v,w));
		ASSERT(!findEdge(w,v));
	#endif
		edge_struct* e;
		
		if(v->_index<w->_index)
			e=new edge_struct(v, w, m_aE.size());
		else
			e=new edge_struct(w, v, m_aE.size());

		e->_data=data;
		m_aE.push_back(e);
		
		v->_aE.push_back(e);
		w->_aE.push_back(e);
		
#ifdef _DEBUG
//		for(int ii=0; ii<m_aV.size(); ii++)
//			check_vertex(*this, ii);
#endif
		return e; 
	}

	node_struct* graph_impl::node(int i) const
	{
		ASSERT(i>=0 && i<m_aV.size());
#ifdef _DEBUG
		check_vertex(*this, i);
#endif
		return m_aV[i];
	}

	edge_struct* graph_impl::edge(int i) const
	{
		ASSERT(i>=0 && i<m_aE.size());
		return m_aE[i];
	}

	edge_struct* graph_impl::findEdge(node_struct* v, node_struct* w)
	{
		edge_struct* e;
		for(int i=0; i<v->_aE.size(); i++)
		{
			e=v->_aE[i];

			if(e->target(v)==w)
				return e;
		}
		return NULL;
	}

	void graph_impl::clear()
	{
		for(int i=0; i<m_aV.size(); i++)
		{
			delete m_aV[i];
		}
		m_aV.resize(0);

		for(int i=0; i<m_aE.size();i++)
		{
			delete m_aE[i];
		}

		m_aE.resize(0);
	}
}// end of namespace TUGL
