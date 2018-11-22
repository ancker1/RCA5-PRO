#pragma once
#include "Node.h"
#include <bitset>
#include <iostream>
class Graph
{
public:
	Graph();
	void AddNode(Node* n);
	Node* GetCurrentNode();		// Returns * cnode ( node at which the robot currently is )
	bool MoveToNode(int v);		// This will move the robot the a specified node (index) - if the return is true the move was valid. This will update bitset<8> visited.
	std::vector<Node*> possible_actions(Node*);	// This will return a vector with a set of valid nodes that can be travelled to.
	char GetVisitedNodes();
	void Init();
	int GetAmountNodes();
	~Graph();
private:
	Node* cnode;
	std::vector<Node*> nodes;
	std::bitset<8> visited;
	int amount_nodes = 0;
};

