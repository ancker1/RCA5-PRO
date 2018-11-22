#include "Node.h"




Node::Node(wchar_t v)
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

wchar_t Node::GetValue()
{
	return value;
}


Node::~Node()
{
}
