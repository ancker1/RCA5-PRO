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

wchar_t Graph::GetVisitedNodes()
{

	unsigned long temp = visited.to_ulong();
	wchar_t ret = static_cast<wchar_t>(temp);
	return ret;
}

void Graph::Init()
{
	cnode = nodes[0];
	visited.reset();
	//DistributeMarbles();
}

int Graph::GetAmountNodes()
{
	return amount_nodes;
}

void Graph::PrintDistribution()
{
	for ( Node* n : nodes )
	{
		std::cout << "Node: " << n->GetValue() << ", marbles: " << n->GetMarbles() <<  std::endl;
	}
}


Graph::~Graph()
{
}

void Graph::DistributeMarbles()
{
	//srand(NULL);
	/*	Generate amount of marbles between or equal to min and max	*/
	int min = 5;
	int max = 20;
	int threshold = max - min + 1;
	int random_num = rand() % threshold;
	marbles_total = random_num;
	/*		Set amount of marbles equal to 0	*/
	for (int i = 0; i < nodes.size(); i++)
		nodes[i]->SetMarbles(0);
	/*	Distribute the marbles randomly	 */
	int distributed_marbles = 0;
	for (;; )
	{
		for (int i = 0; i < nodes.size(); i++)
		{
			random_num = rand() % 101;	// Generate random num between 0 and 100.
			if (nodes[i]->GetProbability() * 100 > random_num)
			{
				nodes[i]->SetMarbles(nodes[i]->GetMarbles() + 1);
				distributed_marbles++;
			}
		}
		if ( distributed_marbles >= marbles_total )
		{
			while ( distributed_marbles > marbles_total )
			{	// Choose a node at random to remove marble from
				random_num = rand() % nodes.size();
				if (nodes[random_num]->GetMarbles())	// Check if there is marble to remove.
				{
					nodes[random_num]->SetMarbles(nodes[random_num]->GetMarbles() - 1);
					distributed_marbles--;
				}
			}
			break;	// Break out for for( ;; ) when the amount of marbles distributed is equal to marbles_total.
		}
	}

}
