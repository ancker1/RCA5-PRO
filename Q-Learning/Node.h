#pragma once
#include <vector>
class Node
{
public:
	Node(int);
	void AddNeighbor(Node* n);
	std::vector<Node*> GetNeighbors();
	int GetValue();
	~Node();
private:
	std::vector<Node*> neighbors;
	int value; 
};

