#include "Node.h"

Node::Node(wchar_t v, float p)
{
	value = v;
	marble_probability = p;
}

void Node::AddNeighbor(Node * n)
{
	neighbors.push_back(n);
}

void Node::AddNeighbor(std::initializer_list<Node*> list)
{
	for ( Node* n : list )
	{
		neighbors.push_back(n);
	}
}

std::vector<Node*> Node::GetNeighbors()
{
	return neighbors;
}

wchar_t Node::GetValue()
{
	return value;
}

void Node::SetMarbles(int amount)
{
	marbles = amount;
}

int Node::GetMarbles()
{
	return marbles;
}

float Node::GetProbability()
{
	return marble_probability;
}


Node::~Node()
{
}
