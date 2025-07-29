#include <iostream>
#include <string>
#include <climits>
#include <queue>
#include <cmath>
#include <cstring>

#define Max_V 100

using namespace std;
class Queue
{
private:
	pair<int,int> *arr;
	int front;
	int rear;
	int size;
public:
	Queue(int size)
	{
		this->size=size;
		arr=new pair<int,int>[size];
		front=0;
		rear=0;
	}
	~Queue()
	{
		delete[] arr;
	}
	bool empty()
	{
		return front==rear;
	}
	void push(int a,int b)
	{
		if(rear<size)
		{
			arr[rear++]=make_pair(a,b);
		}
		else
		{
			cout<<"Queue is Full"<<endl;
		}
	}
	void pop()
	{
		if(!empty())
		{
			front++;
		}
		else
		{
			cout<<"Queue is Empty"<<endl;
		}
	}
	int frontFirst()
	{
		if(!empty())
		{
			return arr[front].first;
		}
		else
		{
			cout<<"Queue is Empty"<<endl;
			return -1;
		}
	}
	int frontSecond()
	{
		if(!empty())
		{
			return arr[front].second;
		}
		else
		{
			cout<<"Queue is Empty"<<endl;
			return -1;
		}
	}
};
class Graph
{
private:
	int V;
	int capacity[Max_V][Max_V];
	int flow[Max_V][Max_V];
	int adj[Max_V][Max_V];
	int adjCount[Max_V];
public:
	Graph(int V)
	{
	   this->V=V;	
	   memset(capacity,0,sizeof(capacity));
	   memset(flow,0,sizeof(flow));
	   memset(adj,0,sizeof(adj));
	   memset(adjCount,0,sizeof(adjCount));
    }
    void addEdge(int u,int v,int capacity)
    {
    	this->capacity[u][v]=capacity;
		//this->flow[u][v]=flow;
		//this->flow[v][u]=-flow;
		adj[u][adjCount[u]++]=v;
		adj[v][adjCount[v]++]=u;
	}
	int Bfs(int s,int t,int parent[])
	{
		memset(parent,-1,sizeof(int)*Max_V);
		parent[s]=-2;
		queue <pair<int,int>> q;
		q.push({s,INT_MAX});
		while(!q.empty())
		{
			int u=q.front().first;
		    int flow_in_path=q.front().second;
		    q.pop();
	
		    for(int i=0;i<adjCount[u];i++)
		    {
			    int v=adj[u][i];
			    int residual_capacity=capacity[u][v]-flow[u][v];
			
			    if(parent[v]==-1 && residual_capacity>0)
			    {
			 	    parent[v]=u;
				    int new_flow=min(flow_in_path,residual_capacity);
				    if(v==t)
				    {
					    return new_flow;
				    }
				    q.push({v,new_flow});
			    }
			}
	    }
		return 0;	
	}
	void Dfs(int u ,bool visited[])
	{
		visited[u]=true;
		for(int i=0;i<adjCount[u];i++)
		{
			int v=adj[u][i];
			if(!visited[v] && capacity[u][v]-flow[u][v]>0)
			{
				Dfs(v,visited);
			}
		}
	}
	int maxFlow(int s,int t)
	{
		int total_flow=0;
		int parent[Max_V];
		
		int flow_in_path;
		 while((flow_in_path=Bfs(s,t,parent)))
		{
			total_flow+=flow_in_path;
			int v=t;
			while(v!=s)
			{
				int u=parent[v];
				flow[u][v]+=flow_in_path;
				flow[v][u]-=flow_in_path;
				v=u;
			}
		}
		return total_flow;
	}
	void minCut(int s)
	{
		bool visited[Max_V];
		memset(visited,false,sizeof(visited));
		Dfs(s,visited);
		
		for(int u=0;u<V;u++)
		{
			if(visited[u])
			{
				for(int i=0;i<adjCount[u];i++)
				{
					int v=adj[u][i];
					if(!visited[v] && capacity[u][v] >0)
					{
						cout<<u<<" - "<<v<<" is the min cut"<<endl;
					}
				}
			}
	    }
	}
};
int main()	
{			
	//3 test cases need to be implemented 2 have been implemented one opposite direction remains;
      Graph g(10);
    g.addEdge(0, 1, 3);
    g.addEdge(0, 3, 4);
    g.addEdge(0, 2, 2);
    g.addEdge(1, 4, 5);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 7, 2);
    g.addEdge(5, 4, 4);
    g.addEdge(5, 8, 1);
    g.addEdge(7, 8, 3);
    g.addEdge(4, 9, 3);
    g.addEdge(8, 9, 5);

    cout << "Max Flow: " << g.maxFlow(0, 9) << endl;
    g.minCut(0);
}
