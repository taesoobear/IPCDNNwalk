#ifndef _TGL_H_
#define _TGL_H_
#if _MSC_VER>1000
#pragma once
#endif
#include <vector>
#include <list>
#include "TList.h" // -tlist dependency will be removed.
#include "../math/mathclass.h"
namespace TGL
{
	// taesoo's implementation of LEDA interface.
struct EmptyData
{
};

struct edge_struct;
struct node_struct
{
	node_struct():_index(-1), _data(NULL){}
	node_struct(int index):_data(NULL) { _index=index;}
	~node_struct()		{}

	int _index;			// internal name (index)

	//original LEDA_deprecated의 경우 OUTGOING=0, INCOMING=1로 표현한다.
	std::vector<edge_struct*> _aIncomingE;	//!< reference array로 초기화 함에 주의
	std::vector<edge_struct*> _aOutgoingE;//!< reference array로 초기화 함에 주의
	void* _data;
};

template <class NodeType, class EdgeType>
class edge;

template <class NodeType, class EdgeType=EmptyData>
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
	NodeType& operator*() const		{ return data();}

	edge<NodeType, EdgeType> inEdge(int index) const;
	edge<NodeType, EdgeType> outEdge(int index) const;

	edge_struct* getOutEdgePtr(int index) const		{ return _outEdge()[index];}
	edge_struct* getInEdgePtr(int index) const		{ return _inEdge()[index];}

	int  outdeg()    const	{ return _outEdge().size(); }
	int  indeg()     const	{ return _inEdge().size(); }
	int  degree()    const	{ return outdeg()+indeg();}

	inline operator node_struct*()					{ return _ptr;}
	friend bool operator==(node const& a, node const& b)
	{
		return a._ptr==b._ptr;
	}

	// almost private:
	const std::vector<edge_struct*>& _outEdge() const	{ return _ptr->_aOutgoingE;}
	const std::vector<edge_struct*>& _inEdge() const		{ return _ptr->_aIncomingE;}

};

struct edge_struct
{
	edge_struct():_index(-1),_s(NULL),_t(NULL),_data(NULL)	{}
	~edge_struct()						{}
	edge_struct(node_struct* v, node_struct* w, int index) {_index=index;_s=v;_t=w;}
	int  _index;          // internal name (index)
	node_struct* _s;             // source node
	node_struct* _t;             // target node
	void* _data;

};

template <class NodeType, class EdgeType=EmptyData>
class edge
{
public:
	edge():_ptr(NULL){}
	edge_struct* _ptr;

	node<NodeType, EdgeType> source()    const { return _ptr->_s;}
	node<NodeType, EdgeType> target()    const { return _ptr->_t;}
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
edge<NodeType, EdgeType> node<NodeType, EdgeType>::inEdge(int index) const
{
	edge<NodeType, EdgeType> e;
	e._ptr=getInEdgePtr(index);
	return e;
}

template <class NodeType, class EdgeType>
edge<NodeType, EdgeType> node<NodeType, EdgeType>::outEdge(int index) const
{
	edge<NodeType, EdgeType> e;
	e._ptr=getOutEdgePtr(index);
	return e;
}

// connect to a graph before use. e.g. graph.connect(array1);
class base_node_array
{
public:
	std::list<base_node_array*> *_listNodeArray;

	base_node_array(){_listNodeArray=NULL;}
	virtual ~base_node_array(){if(_listNodeArray) _listNodeArray->remove(this); }
	virtual int size() const			{ return 0;}

	virtual void update(int newNumNode) {}
};

class base_edge_array
{
public:
	std::list<base_edge_array*> *_listEdgeArray;

	base_edge_array(){ _listEdgeArray=NULL;}
	virtual ~base_edge_array(){if(_listEdgeArray) _listEdgeArray->remove(this); }
	virtual int size() const			{ return 0;}

	virtual void update(int newNumEdge) {}
};

/** 가정: 에지나 node가 추가로 생길수는 있지만 중간에 지워지지는 않는다. 그래프 자료구조에서는 상관없지만, node_array나 edge_array의 구현에서 빠른 indexing 속도와 쉬운 구현을 위해 array를 사용했기 때문. 나중에 이 가정을 없앨 필요가 있으면 아마도 interface는 유지한채로 재구현이 가능할 것이다.
*/
template <class NodeType, class EdgeType=EmptyData>
class graph {

	std::vector<node_struct*> m_aV;              //!< list of all nodes
	std::vector<edge_struct*> m_aE;              //!< list of all edges

	node_struct* newNodeStruct(int index);

	void deleteNodeStruct(node_struct* v);
	void deleteEdgeStruct(edge_struct* e);
public:
	graph();
	virtual ~graph()		{ clear(); }
	virtual void clear();
	void clearEdges();

	int  numNodes() const   { return m_aV.size(); }
	int  numEdges() const   { return m_aE.size(); }

	// if you know the number of nodes to be created, please reserve NodeArray for efficiency
	void reserveNodeArray(int numNodes)		{ m_aV.reserve(numNodes);}
	void reserveEdgeArray(int numEdges)		{ m_aE.reserve(numEdges);}

	/// 새 노드생성과 edge생성은 반드시 newNode, newEdge를 사용할것.
	node<NodeType, EdgeType> newNode();
	edge<NodeType, EdgeType> newEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w);
	edge<NodeType, EdgeType> newEdge(int v_index, int w_index)	{ return newEdge(findNode(v_index), findNode(w_index));}

	node<NodeType, EdgeType> findNode(int index) const { node<NodeType, EdgeType> v; v._ptr=(node_struct*)(m_aV[index]); return v;}

	edge<NodeType, EdgeType> findEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w) const;
	edge<NodeType, EdgeType> findEdge(int v_index, int w_index) const { return findEdge(findNode(v_index), findNode(w_index));}
	edge<NodeType, EdgeType> findEdge(int index) const { edge<NodeType, EdgeType> e; e._ptr=(edge_struct*)(m_aE[index]); return e;}

	inline node_struct* getNodePtr(int index) const	{ return (node_struct*)m_aV[index];}
	inline edge_struct* getEdgePtr(int index) const	{ return (edge_struct*)m_aE[index];}

	virtual void connect(base_node_array& c) const
	{
		if(c._listNodeArray )
			c._listNodeArray->remove(&c);

		m_listNodeArray.push_back(&c);
		c._listNodeArray=&m_listNodeArray;
		c.update(numNodes());
	}


	virtual void connect(base_edge_array& c) const
	{
		if(c._listEdgeArray )
			c._listEdgeArray->remove(&c);

		m_listEdgeArray.push_back(&c);
		c._listEdgeArray=&m_listEdgeArray;
		c.update(numEdges());
	}

protected:
	mutable std::list<base_node_array*> m_listNodeArray;
	mutable std::list<base_edge_array*> m_listEdgeArray;

};

template <class T>
class edge_array : public base_edge_array
{
	std::vector<T> mArray;
public:
	edge_array():base_edge_array(){}
	~edge_array(){}

	virtual int size() const				{ return mArray.size();}
	virtual void update(int newNumEdge)		{ mArray.resize(newNumEdge); }
	T & operator[](edge_struct* E) const	 { return (T&)(mArray[E->_index]);}

};

template <class T>
class node_array : public base_node_array
{
	std::vector<T> mArray;
public:
	node_array():base_node_array(){}
	template <class NodeType, class EdgeType>
	node_array(graph<NodeType, EdgeType>& g):base_node_array(){g.connect(*this);}
	~node_array(){}

	virtual void update(int newNumNode)		{ return mArray.resize(newNumNode);}
	T & operator[](node_struct* V) const	{ return (T&)(mArray[V->_index]);}
};

class edge_int_array : public intvectorn, public base_edge_array
{
	public:
	edge_int_array ():intvectorn(),base_edge_array(){}
	~edge_int_array (){}

	virtual void update(int newNumEdge) { resize(newNumEdge);}
	int& operator[](edge_struct* E) const		{ return value(E->_index);}
};

class node_int_array : public intvectorn, public base_node_array
{
	public:
	node_int_array ():intvectorn(),base_node_array(){}
	~node_int_array (){}

	virtual void update(int newNumNode)			{ resize(newNumNode); }
	int& operator[](node_struct* V) const		{ return value(V->_index);}
};

template <class T, class TE>
bool BELLMAN_FORD(const graph<T,TE>& G, node<T,TE> s, const edge_array<float>& cost,
                                                node_array<float>& dist,
                                                node_array<edge<T,TE> >& pred ) ;

template <class T, class TE>
void PATH(node<T,TE> s, node<T,TE> t, const node_array<edge<T,TE> >& pred, TList<node<T,TE> >& path);


template <class T, class TE>
void PATH_EDGE(node<T,TE> s, node<T,TE> t, const node_array<edge<T,TE> >& pred, TList<edge<T,TE> >& path);


template <class T, class TE>
void BFS(const graph<T,TE>& G, node<T,TE> v, node_array<int>& dist);


template <class T, class TE>
void DFS(const graph<T,TE>& G, node<T,TE> v, node_array<bool>& reached, TList<node<T,TE> >& L);


template <class T, class TE>
void DRAW(const graph<T,TE>& G, const node_array<TString>& name, const char* filename);


template <class T, class TE>
void DRAW(const graph<T,TE>& G, const node_array<TString>& name, const edge_array<TString>& nameE, const char* filename);
}// end namespace

// TGL_for_... 과 TGL_end_for 쌍을 사용할 것-> 변경됨. TGL_end_for가 없어도 동작함..
#define TGL_end_for
#define TGL_for_all_node TGL_for_all_nodes
#define TGL_forall_nodes TGL_for_all_nodes
#define TGL_for_all_edge TGL_for_all_edges
#define TGL_for_all_adj_edges TGL_for_outgoing_edges

#define TGL_for_all_nodes(v,g)\
	for(int _tgl_i=0; _tgl_i<(g).numNodes() &&((v)._ptr=(g).getNodePtr(_tgl_i))!=NULL; _tgl_i++)

#define TGL_for_all_edges(e,g)\
	for(int _tgl_i=0; _tgl_i<(g).numEdges() && ((e)._ptr=(g).getEdgePtr(_tgl_i))!=NULL; _tgl_i++)

#define TGL_for_outgoing_edges(e,v)\
	for(int _tgl_i=0; _tgl_i<(v).outdeg() && ((e)._ptr=(v).getOutEdgePtr(_tgl_i))!=NULL; _tgl_i++)

/*
forall_adj_nodes(v, w) { ... }
iterates over all nodes v that are adjacent to the node w. In the case of a directed graph, these are all v for which there is an edge (w,v) in the graph. In the undirected case, these are all v for which there is an edge (w,v) or an edge (v,w). */

#define TGL_forall_adj_nodes(v,w)\
	for(int _tgl_i=0; _tgl_i<(w).outdeg() && ((v)._ptr=(w).getOutEdgePtr(0)->_t)!=NULL; _tgl_i++)


#include <queue>
#include <stack>

// 절대로 이 파일을 프로젝트 리스트에 넣지 말것.
namespace TGL
{

template <class T, class TE>
graph<T, TE>::graph()
{
}

template <class T, class TE>
node_struct* graph<T, TE>::newNodeStruct(int index)
{
	node_struct* v=new node_struct(index);
	v->_data=new T();
	return v;
}


template <class T, class TE>
void graph<T, TE>::deleteNodeStruct(node_struct* v)
{
	delete (T*)(v->_data);
	delete v;
}

template <class T, class TE>
void graph<T, TE>::deleteEdgeStruct(edge_struct* e)
{
	delete (TE*)(e->_data);
	delete e;
}


template <class T, class TE>
node<T,TE> graph<T, TE>::newNode()
{
	m_aV.resize(m_aV.size()+1);
	m_aV.back()=newNodeStruct(m_aV.size()-1);

	for(std::list<base_node_array*>::iterator i=m_listNodeArray.begin();
		i!=m_listNodeArray.end();
		++i)
	{
		(*i)->update(numNodes());
	}
	return m_aV.back();
}

template <class T, class TE>
void graph<T, TE>::clearEdges()
{
	for(int i=0; i<m_aE.size();i++)
		deleteEdgeStruct(m_aE[i]);

	m_aE.resize(0);

	for(int i=0; i<m_aV.size(); i++)
	{
		m_aV[i]->_aIncomingE.resize(0);
		m_aV[i]->_aOutgoingE.resize(0);
	}
}

template <class T, class TE>
void graph<T, TE>::clear()
{
	for(int i=0; i<m_aV.size(); i++)
		deleteNodeStruct(m_aV[i]);
	m_aV.resize(0);

	for(int i=0; i<m_aE.size();i++)
		deleteEdgeStruct(m_aE[i]);

	m_aE.resize(0);

	for(std::list<base_node_array*>::iterator i=m_listNodeArray.begin();
	i!=m_listNodeArray.end();
	++i)
	{
		if((*i)->_listNodeArray==&m_listNodeArray) (*i)->_listNodeArray=NULL;
	}

	for(std::list<base_edge_array*>::iterator i=m_listEdgeArray.begin();
		i!=m_listEdgeArray.end();
		++i)
	{
		if((*i)->_listEdgeArray==&m_listEdgeArray) (*i)->_listEdgeArray=NULL;
	}

	m_listNodeArray.clear();
	m_listEdgeArray.clear();
}


template <class T, class TE>
edge<T,TE> graph<T, TE>::newEdge(node<T,TE> v, node<T,TE> w)
{
#ifdef _DEBUG
	ASSERT(findNode(v.index())==v);
	ASSERT(findNode(w.index())==w);
	/* In general, duplicated edges are allowed. If not, test by yourself.
	//ASSERT(!findEdge(v,w));
	*/
#endif
	m_aE.resize(m_aE.size()+1);

	edge_struct* e=new edge_struct(v, w, m_aE.size()-1);
	e->_data=new TE();
	m_aE.back()=e;


	v._ptr->_aOutgoingE.push_back(e);
	w._ptr->_aIncomingE.push_back(e);


	for(std::list<base_edge_array*>::iterator i=m_listEdgeArray.begin();
		i!=m_listEdgeArray.end();
		++i)
	{
		(*i)->update(numEdges());
	}

	edge<T,TE> ee;
	ee._ptr=e;
	return ee;
}


template <class T, class TE>
edge<T,TE> graph<T, TE>::findEdge(node<T,TE> v, node<T,TE> w) const
{
	edge<T,TE> e;
	TGL_for_outgoing_edges(e,v)
		if(e.target()==w)
			return e;
	TGL_end_for;

	e._ptr=NULL;
	return e;
}

/*

template <class T, class TE>
node<T,TE> graph<T, TE>::findNode(int index) const
{
	TGL::node<T,TE> v;
	TGL_for_all_nodes(v, *this)
	{
		if(v.index()==index)
			return v;
	}

	v._ptr=NULL;
	return v;
}*/



template <class T, class TE>
bool BELLMAN_FORD(const graph<T, TE>& G, node<T,TE> s, const edge_array<float>& cost,
                                                node_array<float>& dist,
                                                node_array<edge<T,TE> >& pred )

/* single source shortest paths from s using a queue (breadth first search)
   computes for all nodes v:
   a) dist[v] = cost of shortest path from s to v
   b) pred[v] = predecessor edge of v in shortest paths tree
*/
{
	node_array<int> count;
	G.connect(count);
	G.connect(dist);
	G.connect(pred);

	int n = G.numNodes();

	TList<node<T,TE> > Q;

	node<T,TE> u,v;
	edge<T,TE> e;

	TGL_for_all_nodes(v,G)
	{
		pred[v] ._ptr= 0;
		dist[v] = FLT_MAX;
	}


	dist[s] = 0;
	Q.pushBack(s);

	while(! Q.empty() )
	{
		u = Q.popBack();

		if (++count[u] > n) return false;   // negative cycle

		float du = dist[u];

		TGL_for_all_adj_edges(e,u)
		{
			v = e.target();
			float c = du + cost[e];
			if (c < dist[v])
			{
				dist[v] = c;
				pred[v] = e;
				if (!Q.member(v)) Q.pushBack(v);
			}
		}
		TGL_end_for;
	}
  return true;
}


template <class T, class TE>
void PATH(node<T,TE> s, node<T,TE> t, const node_array<edge<T,TE> >& pred, TList<node<T,TE> >& path)
{
	path.init();

	for(node<T,TE> n=t; n!=s; n=pred[n].source())
		path.pushFront(n);
	path.pushFront(s);
}


template <class T, class TE>
void PATH_EDGE(node<T,TE> s, node<T,TE> t, const node_array<edge<T,TE> >& pred, TList<edge<T,TE> >& path)
{
	path.init();
	edge<T,TE> e;
	for(e=pred[t]; e.source()!=s; e=pred[e.source()])
		path.pushFront(e);
	path.pushFront(e);
}


template <class T, class TE>
void DRAW(const graph<T, TE>& G, const node_array<TString>& name, const char* filename)
{

	TString tfilename;
	tfilename.format("../graph/%s", filename);
	FILE* dotfile;
	dotfile=fopen(tfilename+".dot", "wt");
	Msg::verify(dotfile, "%s.dot open failed", tfilename.ptr());

	fprintf(dotfile," digraph G {\n");
	node<T,TE> v;
	edge<T,TE> e;
	TGL_for_all_node(v, G)
	{
		fprintf(dotfile,"%d [label=\"%s\"];\n", v.index(), name[v].ptr());
	}
	TGL_end_for;

	TGL_for_all_edge (e, G)
	{
		fprintf(dotfile,"%d -> %d;\n", e.source().index(), e.target().index());
	}
	TGL_end_for;

	fprintf(dotfile,"}\n");
	fclose(dotfile);

	dotfile=fopen(tfilename+".bat", "wt");
	Msg::verify(dotfile, "%s.bat open failed", tfilename.ptr());

	fprintf(dotfile,"dot -Tjpg %s.dot -o %s.jpg\n", filename, filename);
	fprintf(dotfile,"dot -Tps %s.dot -o %s.ps\n", filename, filename);
	fclose(dotfile);
}


template <class T, class TE>
void DRAW(const graph<T, TE>& G, const node_array<TString>& name, const edge_array<TString>& nameE, const char* filename)
{
	TString tfilename;
	tfilename.format("../graph/%s", filename);
	FILE* dotfile;
	dotfile=fopen(tfilename+".dot", "wt");
	Msg::verify(dotfile, "%s.dot open failed", tfilename.ptr());

	fprintf(dotfile," digraph G {\n");
	node<T,TE> v;
	edge<T,TE> e;

	TGL_for_all_node(v, G)
	{
		fprintf(dotfile,"%d [label=\"%s\"];\n", v.index(), name[v].ptr());
	}
	TGL_end_for;

	TGL_for_all_edge (e, G)
	{
		fprintf(dotfile,"%d -> %d [label=\"%s\"];\n", e.source().index(), e.target().index(), nameE[e].ptr());
	}
	TGL_end_for;

	fprintf(dotfile,"}\n");
	fclose(dotfile);

	dotfile=fopen(tfilename+".bat", "wt");
	Msg::verify(dotfile, "%s.bat open failed", tfilename.ptr());

	fprintf(dotfile,"dot -Tjpg %s.dot -o %s.jpg\n", filename, filename);
	fprintf(dotfile,"dot -Tps %s.dot -o %s.ps\n", filename, filename);
	fclose(dotfile);
}


template <class T, class TE>
void DFS(const graph<T, TE>& G, node<T,TE> v, node_array<bool>& reached, TList<node<T,TE> >& L)
{
	std::stack<node<T,TE> > S;
	node<T,TE> w;

	if ( !reached[v] ) {
	reached[v] = true;
	S.push(v);
	}

	while ( !S.empty() ) {

		v = S.top();
		S.pop();
		L.pushBack(v);
		TGL_forall_adj_nodes(w,v)
		{
			if ( !reached[w] ) {
				reached[w] = true;
				S.push(w);
			}
		}
		TGL_end_for;
	}
}

template <class T, class TE>
void BFS(const graph<T, TE>& G, node<T,TE> v, node_array<int>& dist)
{
	std::queue<node<T,TE> > Q;
	G.connect(dist);
	node<T,TE> w;

	TGL_forall_nodes(w,G)
		dist[w] = -1;
	TGL_end_for;

	dist[v] = 0;
	Q.push(v);

	while ( !Q.empty() )
	{
		v = Q.front();
		Q.pop();
		TGL_forall_adj_nodes(w,v)
			if (dist[w] < 0) {
				Q.push(w);
				dist[w] = dist[v]+1;
			}
		TGL_end_for;
	}
}

}
#endif
