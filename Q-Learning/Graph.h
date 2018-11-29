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
	wchar_t GetVisitedNodes();	// Not used
	void Init();	// Will reset the graph: set cnode at nodes[0], redistribution of marbles.
	int GetAmountNodes();
	void PrintDistribution();
	~Graph();
private:
	void DistributeMarbles();
	/*
		Used for distribution of marbles
	*/
	int marbles_total;
	int amount_nodes = 0;
	/*
		Collection of nodes in the graph
	*/
	Node* cnode;
	std::vector<Node*> nodes;
	std::bitset<16> visited;	// Not used
	
};

