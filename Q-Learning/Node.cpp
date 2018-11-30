#include "Node.h"

Node::Node(wchar_t v, float m, float s)
{
	value = v;
	mean = m;
	std_deviation = s;
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

float Node::GetMean()
{
	return mean;
}

float Node::GetStdDev()
{
	return std_deviation;
}

float Node::GetProbability()
{
	return marble_probability;
}


Node::~Node()
{
}
