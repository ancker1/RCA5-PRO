#include "Graph.h"



Graph::Graph()
{
}

void Graph::AddNode(Node * n)
{
	nodes.push_back(n);
	amount_nodes++;
}

Node * Graph::GetCurrentNode()
{
	return cnode;
}

bool Graph::MoveToNode(int v)
{
	std::vector<Node*> cNeighbors = cnode->GetNeighbors();
	for (int i = 0; i < cNeighbors.size(); i++)
	{
		if (cNeighbors[i]->GetValue() == v)
		{
			cnode = cNeighbors[i];
			visited[i] = 1;
			return true;
		}
	}
	return false;
}

std::vector<Node*> Graph::possible_actions(Node* n)
{
	return n->GetNeighbors();
}

char Graph::GetVisitedNodes()
{

	unsigned long temp = visited.to_ulong();
	char ret = static_cast<char>(temp);
	return ret;
}

void Graph::Init()
{
	cnode = nodes[0];
	visited.reset();
}

int Graph::GetAmountNodes()
{
	return amount_nodes;
}


Graph::~Graph()
{
}
