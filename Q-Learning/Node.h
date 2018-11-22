#pragma once
#include <vector>
class Node
{
public:
	Node(wchar_t);
	void AddNeighbor(Node* n);
	std::vector<Node*> GetNeighbors();
	wchar_t GetValue();
	~Node();
private:
	std::vector<Node*> neighbors;
	int value; 
};

