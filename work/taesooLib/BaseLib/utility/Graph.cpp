#include "../baselib.h"
#include "Graph.h"
#include "TList.h" // -tlist dependency will be removed.
#include <functional>
#include <queue>
using namespace std;

//https://www.geeksforgeeks.org/bellman-ford-algorithm-dp-23/

// A recursive function that finds and prints strongly connected
// components using DFS traversal
// u --> The vertex to be visited next
// disc[] --> Stores discovery times of visited vertices
// low[] -- >> earliest visited vertex (the vertex with minimum
//             discovery time) that can be reached from subtree
//             rooted with current vertex
// *st -- >> To store all the connected ancestors (could be part
//         of SCC)
// stackMember[] --> bit/index array for faster check whether
//                 a node is in stack
void TGL::Graph::SCCUtil(int u, int disc[], int low[], stack<int> *st,
                    bool stackMember[], vecIntvectorn& components)
{
    // A static variable is used for simplicity, we can avoid use
    // of static variable by passing a pointer.
    static int time = 0;
 
    // Initialize discovery time and low value
    disc[u] = low[u] = ++time;
    st->push(u);
    stackMember[u] = true;
 
    // Go through all vertices adjacent to this
	auto& adj=outEdges(u);
    for (auto i = adj.begin(); i != adj.end(); ++i)
    {
        int v = (*i).target(); // v is current adjacent of 'u'
 
        // If v is not visited yet, then recur for it
        if (disc[v] == -1)
        {
            SCCUtil(v, disc, low, st, stackMember, components);
 
            // Check if the subtree rooted with 'v' has a
            // connection to one of the ancestors of 'u'
            // Case 1 (per above discussion on Disc and Low value)
            low[u] = min(low[u], low[v]);
        }
 
        // Update low value of 'u' only of 'v' is still in stack
        // (i.e. it's a back edge, not cross edge).
        // Case 2 (per above discussion on Disc and Low value)
        else if (stackMember[v] == true)
            low[u] = min(low[u], disc[v]);
    }
 
    // head node found, pop the stack and print an SCC
    int w = 0; // To store stack extracted vertices
    if (low[u] == disc[u])
    {
        while (st->top() != u)
        {
            w = (int) st->top();
            //cout << w << " ";
			components.back().push_back(w);
            stackMember[w] = false;
            st->pop();
        }
        w = (int) st->top();
		components.back().push_back(w);
		components.resize(components.size()+1);
		components.back().setSize(0);
        stackMember[w] = false;
        st->pop();
    }
}

void TGL::Graph::_updateE() const
{
	if(!_E_valid)
	{
		int c=0;
		for(auto const& adj :G)
			c+=adj.size();

		_E.resize(c);

		c=0;
		for(int v=0; v<G.size(); v++)
		{
			auto const& adj=outEdges(v);

			for(int j=0; j<adj.size(); j++)
				_E[c+j]=EdgeIterator(v, j);

			c+=adj.size();
		}

		_E_valid=true;
	}
}
void TGL::Graph::SCC(vecIntvectorn& components)
{
	components.resize(1);
	components[0].setSize(0);
	int V=numNodes();
    int *disc = new int[V];
    int *low = new int[V];
    bool *stackMember = new bool[V];
    stack<int> *st = new stack<int>();
 
    // Initialize disc and low, and stackMember arrays
    for (int i = 0; i < V; i++)
    {
        disc[i] = -1;
        low[i] = -1;
        stackMember[i] = false;
    }
 
    // Call the recursive helper function to find strongly
    // connected components in DFS tree with vertex 'i'
    for (int i = 0; i < V; i++)
        if (disc[i] == -1)
            SCCUtil(i, disc, low, st, stackMember, components);
	delete [] disc;
	delete [] low;
	delete [] stackMember;
	delete st;
	components.resize(components.size()-1);
}

void TGL::Graph::getCost(vectorn& edge_cost) const
{
	edge_cost.resize(numEdges());
	int c=0;
	for(auto const& e: _E)
	{
		edge_cost[c++]=e.edge(this).cost();
	}
}
void TGL::Graph::setCost(vectorn const& edge_cost) 
{
	_updateE();
	int c=0;
	for(auto const& e: _E)
	{
		e.edge(this).setCost(edge_cost[c++]);
	}
}
void TGL::Graph::BELLMAN_FORD(int s, vectorn const& edge_cost, vectorn& node_dist)
{
	vectorn backup;
	getCost(backup);
	setCost(edge_cost);

	BELLMAN_FORD(s,node_dist);

	setCost(backup);
}
void TGL::Graph::BELLMAN_FORD(int s, vectorn& node_dist)
{

	std::vector<int> count, pred;
	std::vector<float> dist;


	int n = numNodes();
	count.resize(numNodes());
	pred.resize(numNodes());
	dist.resize(numNodes());

	TList<int > Q;

	int u,v;
	Edge e;


	for(int i=0; i<n; i++)
	{
		count[i]=0;
		pred[i] =-1;
		dist[i] = float(INT_MAX-1000);
	}


	dist[s] = 0;
	Q.pushBack(s);

	while(! Q.empty() )
	{
		u = Q.popBack();

		if (++count[u] > n) { Msg::error("negative cycle"); return ;  } // negative cycle

		float du = dist[u];

        for(auto e: G[u])    
		{
			v = e.target();
			float c = du + e.cost();
			if (c < dist[v])
			{
				dist[v] = c;
				pred[v] = u;
				if (!Q.member(v)) Q.pushBack(v);
			}
		}
	}

	node_dist.resize(dist.size());
	for(int i=0; i<dist.size(); i++)
		node_dist[i]=dist[i];
}

typedef std::pair<float, int> pfi;
void TGL::Graph::Dijkstra(int source, vectorn& node_dist)
{
	std::priority_queue<pfi, std::vector<pfi>, greater<pfi> > Q;   // min heap 

	int N=numNodes();
	std::vector<float> Dist;
    Dist.assign(N,float(INT_MAX-1000)); 

    Dist[source] = 0.f; 
    Q.push(std::make_pair(0.f,source)); 
    while(!Q.empty()){ 
        int u = Q.top().second; 
        Q.pop(); 
        for(auto &c : G[u]){ 
            int v = c.target(); 
            float w = c.cost(); 
            if(Dist[v] > Dist[u]+w){ 
                Dist[v] = Dist[u]+w; 
                Q.push(std::make_pair(Dist[v],v)); 
            } 
        } 
    } 
	node_dist.resize(Dist.size());
	for(int i=0; i<Dist.size(); i++)
		node_dist[i]=Dist[i];
}
void TGL::Graph::Dijkstra(int source, vectorn& node_dist, intvectorn& pred)
{
	std::priority_queue<pfi, std::vector<pfi>, greater<pfi> > Q;   // min heap 

	int N=numNodes();
	std::vector<float> Dist;
    Dist.assign(N,float(INT_MAX-1000)); 
	pred.resize(N);
	pred.setAllValue(-1);

    Dist[source] = 0.f; 
    Q.push({0.f,source}); 
    while(!Q.empty()){ 
        int u = Q.top().second; 
        Q.pop(); 
        for(auto &c : G[u]){ 
            int v = c.target(); 
            float w = c.cost(); 
            if(Dist[v] > Dist[u]+w){ 
                Dist[v] = Dist[u]+w; 
                Q.push({Dist[v],v}); 
				pred[v]=u;
            } 
        } 
    } 
	node_dist.resize(Dist.size());
	for(int i=0; i<Dist.size(); i++)
		node_dist[i]=Dist[i];
}
int TGL::Graph::EdgeIterator::target(const Graph* g) const
{
	return g->G[_s][_index].target();
}
TGL::Graph::Edge& TGL::Graph::EdgeIterator::edge(Graph* g) const
{
	return g->G[_s][_index];
}
const TGL::Graph::Edge& TGL::Graph::EdgeIterator::edge(const Graph* g) const
{
	return g->G[_s][_index];
}

void TGL::Graph::DRAW(TStrings const& name, const char* filename)
{
	TString tfilename;
	tfilename.format("../graph/%s", filename);
	FILE* dotfile;
	dotfile=fopen(tfilename+".dot", "wt");
	Msg::verify(dotfile, "%s.dot open failed", tfilename.ptr());

	fprintf(dotfile," digraph G {\n");

	for (int v=0; v<numNodes(); v++)
	{
		fprintf(dotfile,"%d [label=\"%s\"];\n", v, name[v].ptr());
	}

	for(int e=0; e<numEdges(); e++)
	{
		fprintf(dotfile,"%d -> %d;\n", source(e), target(e));
	}

	fprintf(dotfile,"}\n");
	fclose(dotfile);

	dotfile=fopen(tfilename+".bat", "wt");
	Msg::verify(dotfile, "%s.bat open failed", tfilename.ptr());

	fprintf(dotfile,"dot -Tjpg %s.dot -o %s.jpg\n", filename, filename);
	fprintf(dotfile,"dot -Tps %s.dot -o %s.ps\n", filename, filename);
	fclose(dotfile);
}



/*1.Using priority queue container in C++ STL.

// Time complexity : O(ElogV) 
#include <bits/stdc++.h> 
using namespace std; 
 
typedef vector<int> vi; 
typedef pair<int,int> pii; 
typedef vector< pii > vii; 
#define INF 0x3f3f3f3f 
 
vii *G;   // Graph 
vi Dist;  // for storing the distance of every other node from source. 
void Dijkstra(int source, int N) { 
    priority_queue<pii, vector<pii>, greater<pii> > Q;   // min heap 
    Dist.assign(N,INF); 
    Dist[source] = 0; 
    Q.push({0,source}); 
    while(!Q.empty()){ 
        int u = Q.top().second; 
        Q.pop(); 
        for(auto &c : G[u]){ 
            int v = c.second; 
            int w = c.first; 
            if(Dist[v] > Dist[u]+w){ 
                Dist[v] = Dist[u]+w; 
                Q.push({Dist[v],v}); 
            } 
        } 
    } 
} 
int main() { 
    int N, M, u, v, w, source;  // N-total no of nodes, M-no. of edges,  
    cin >> N >> M;              // u,v and w are the end vertices and the weight associated with an edge 
    G = new vii[N+1]; 
     
    for(int i=0;i<M;++i){ 
        cin >> u >> v >> w; 
        G[u].push_back({v,w}); 
        G[v].push_back({u,w}); 
    } 
    cin >> source; 
    Dijkstra(source,N); 
     
    for(int i=0;i<N;i++) 
        cout<<Dist[i]<<" "; 
    cout<<endl; 
     
    return 0; 
} 
2. Using set container in C++ STL.

//Dijkstra algorithm for Undirected Graph, time complexity : O(|E|.log|V|)  
#include <bits/stdc++.h> 
using namespace std; 
typedef vector<int> vi; 
typedef pair<int,int> pii; 
typedef vector< pii > vii; 
#define INF 0x3f3f3f3f 
vii *G;    // Graph 
vi D;      // distance vector for storing min distance from the source. 
void dijkstra(int source, int N) { 
    D.assign(N,INF); 
    D[source] = 0; 
    set<pii> Q; 
    Q.insert({0,source}); 
    while(!Q.empty()) { 
        auto top = Q.begin(); 
        int u = top->second; 
        Q.erase(top); 
        for(auto next: G[u])    { 
            int v = next.second, weight = next.first; 
            if( D[v] > D[u] + weight) { 
                if(Q.find( {D[v], v} ) != Q.end()) 
                Q.erase(Q.find( {D[v], v} )); 
                D[v] = D[u] + weight; 
                Q.insert( {D[v], v} ); 
            } 
        } 
    } 
} 
int main(){ 
    int N,M,source,u,v,w; 
    cin >> N >> M >> source;  
//Input the number of nodes(0 based), number of edges and the source vertex. 
    G = new vii[N]; 
    for(int i=0;i<M;i++){ 
        cin >> u >> v >> w;  
//Input the starting vertex of the edge, the ending vertex and the cost of the edge. 
        G[u].push_back({v,w}); 
        G[v].push_back({u,w}); 
    } 
    dijkstra(source, N);  
 
    for(int i=0;i<N;i++) 
        cout<<D[i]<<" "; 
    cout<<endl; 
} 
*/


void TGL::Graph::minimum_spanning_tree(intvectorn& sources, intvectorn& targets) const
{
	float INF = 9999999;
	// number of vertices in graph
	int N = numNodes();
	// creating graph by adjacency matrix method
	std::vector<bool> selected_node;
	selected_node.assign(N, false);

	int no_edge = 0;

	selected_node[0] = true;

	// printing for edge and weight
	int a, b;
	while (no_edge < N - 1) {

		float minimum = INF;
		int a = 0;
		int b = 0;
		for (int m =0; m<N; m++)
		{
			if (selected_node[m] ){
				for(int ii=0, iin=outdeg(m); ii<iin; ii++)
				{
					auto& e=G[m][ii];
					int n=e.target();

					if ((!selected_node[n]) ){
						// not in selected and there is an edge
						if (minimum > e.cost()){
							minimum = e.cost();
							a = m;
							b = n;
						}
					}
				}
			}
		}
		sources.pushBack(a);
		targets.pushBack(b);
		selected_node[b] = true;
		no_edge = no_edge+1;
	}
}
