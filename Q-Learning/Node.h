#pragma once
#include <vector>
#include <initializer_list>
class Node
{
public:
	Node(wchar_t, float mean, float std_deviation);
	void AddNeighbor(Node* n);
	void AddNeighbor(std::initializer_list<Node*>);
	std::vector<Node*> GetNeighbors();
	wchar_t GetValue();
	void SetMarbles(int amount);
	int GetMarbles();
	float GetMean();
	float GetStdDev();
	float GetProbability();
	~Node();
private:
	std::vector<Node*> neighbors;
	/* Used for self-made probability */
	float marble_probability;
	/* Used for normal distribution */
	float mean;
	float std_deviation;
	int marbles = 0;
	int value;
};

