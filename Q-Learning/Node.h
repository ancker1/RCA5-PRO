#pragma once
#include <vector>
#include <initializer_list>
class Node
{
public:
	Node(wchar_t, float prob);
	void AddNeighbor(Node* n);
	void AddNeighbor(std::initializer_list<Node*>);
	std::vector<Node*> GetNeighbors();
	wchar_t GetValue();
	void SetMarbles(int amount);
	int GetMarbles();
	float GetProbability();
	~Node();
private:
	std::vector<Node*> neighbors;
	float marble_probability;
	int marbles = 0;
	int value;
};

