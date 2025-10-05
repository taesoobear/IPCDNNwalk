#ifndef _TGL_GRAPH_H_
#define _TGL_GRAPH_H_
#if _MSC_VER>1000
#pragma once
#endif


//#include "TGL.h" -- this dependency has been removed.
#include <stack>

namespace TGL
{
	// for lua scripts and simple graphs.
	class Graph
	{
		public:
		class Edge
		{
				int _target;
				float _cost;
			public:
				Edge():_target(-1),_cost(0.f){}
				Edge(int t, float c):_target(t),_cost(c){}

				inline int target() const { return _target;}
				inline float cost() const { return _cost;}
				void setCost(float c) { _cost=c;}
		};
		class EdgeIterator
		{
				int  _index, _s;
			public:
				EdgeIterator():_s(-1),_index(-1)	{}
				EdgeIterator(int v, int i) {_index=i;_s=v;}
				inline int source() const { return _s;}
				int target(const Graph* g) const;
				Edge & edge(Graph* g) const;
				const Edge & edge(const Graph* g) const;
		};


		typedef std::vector<Edge> Edges;

		std::vector<Edges> G;
		private:
		mutable std::vector<EdgeIterator> _E;
		mutable bool _E_valid;

		inline static bool Contains( Edges const& Vec, int Element ) 
		{
			for(auto const& e: Vec)
				if(e.target()==Element)
					return true;
			return false;
		}
		friend class EdgeIterator;
		public:

		using vecIntvectorn=std::vector<intvectorn>;
		using vecVectorn=std::vector<vectorn>;

		Graph(int V) // No. of vertices
		{
			G.resize(V);
			_E_valid=false; // cache needs to be updated.
		}

		void resize(int numN) { G.resize(numN); _E_valid=false;}
		void getCost(vectorn& edge_cost) const;
		void setCost(vectorn const& edge_cost) ;
		inline void clear() 			{ G.clear();_E_valid=false;}
		inline void clearEdges() 		{ int V=G.size(); clear(); G.resize(V);}

		inline void reserveNodeArray(int numNodes) { G.reserve(numNodes);}
		inline void reserveEdgeArray(int numEdges) { 
			_E_valid=false;
			int nAvgE=numEdges/numNodes()+1;
			for(int i=0; i<numNodes(); i++)
				G[i].reserve(nAvgE);
		}

		inline void newNode() 			{ G.resize(G.size()+1);}
		// function to add an edge to graph
		inline void addEdge(int v, int w, float cost=1.f) { 
			G[v].push_back(Edge(w, cost));
			_E_valid=false;
		} 

		void _updateE() const;
		inline int  numNodes() const   { return G.size();}
		inline int  numEdges() const   { _updateE(); return _E.size();}

		inline int outdeg(int v) const { return G[v].size();}
		inline Edges const& outEdges(int v) const { return G[v];}

		inline bool hasEdge(int v, int w) const { return Contains(G[v], w);}

		int source(int iEdge) const { _updateE(); return _E[iEdge].source();}
		int target(int iEdge) const { _updateE(); return _E[iEdge].target(this);}
		float cost(int iEdge) const { _updateE(); return _E[iEdge].edge(this).cost();}

		void DRAW(TStrings const& node_names, const char* filename);
		inline void DRAW(const char* filename)
		{
			TStrings names;
			names.resize(numNodes());
			for(int i=0; i<numNodes(); i++)
				names[i].format("%d", i);
			DRAW( names, filename);
		}
		void SCC(vecIntvectorn& components);
		/*
		 * see TGL.h.
		void BFS(int v, intvectorn& dist);
		void DFS(int v, boolN& reached, intvectorn& nodesL);
		*/
		void BELLMAN_FORD(int startNode, vectorn& node_dist);
		void BELLMAN_FORD(int startNode, vectorn const& edge_cost, vectorn& node_dist);


		// Using priority queue container in C++ STL.
		void Dijkstra(int startNode, vectorn& node_dist);
		void Dijkstra(int startNode, vectorn& node_dist, intvectorn& pred);

		void minimum_spanning_tree(intvectorn& sources, intvectorn& targets) const;

		private:
		// A Recursive DFS based function used by SCC()
		void SCCUtil(int u, int disc[], int low[], std::stack<int> *st, bool stackMember[], vecIntvectorn& components);
	};
};

#endif
