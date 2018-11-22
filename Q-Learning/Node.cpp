#include "Node.h"




Node::Node(int v)
{
	value = v;
}

void Node::AddNeighbor(Node * n)
{
	neighbors.push_back(n);
}

std::vector<Node*> Node::GetNeighbors()
{
	return neighbors;
}

int Node::GetValue()
{
	return value;
}


Node::~Node()
{
}
