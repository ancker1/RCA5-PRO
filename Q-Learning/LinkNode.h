#pragma once
#include <iostream>
#include <string>
using namespace std;
class LinkNode
{
public:
	LinkNode();
	LinkNode(wchar_t room, wchar_t visited, float data);
	void SetNext(LinkNode * Node);
	LinkNode* GetNext();
	float GetData();
	wchar_t GetRoom();
	wchar_t GetVisited();
	~LinkNode();
private:
	wchar_t room;
	wchar_t visited;
	float data;
	LinkNode * next = nullptr;
};

