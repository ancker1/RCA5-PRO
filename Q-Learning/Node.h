#pragma once
#include <vector>
#include <initializer_list>
class Node
{
public:
	Node(wchar_t);
	void AddNeighbor(Node* n);
	void AddNeighbor(std::initializer_list<Node*>);
	std::vector<Node*> GetNeighbors();
	wchar_t GetValue();
	~Node();
private:
	std::vector<Node*> neighbors;
	int value; 
};

