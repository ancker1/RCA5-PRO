#pragma once
#include <iostream>
#include <string>
using namespace std;
class LinkNode
{
public:
	LinkNode();
	LinkNode(wchar_t room, wchar_t visited, wchar_t action, float data);
	void SetNext(LinkNode * Node);
	LinkNode* GetNext();
	float GetData();
	wchar_t GetRoom();
	wchar_t GetVisited();
	wchar_t GetAction();
	void SetData(float d);
	~LinkNode();
private:
	wchar_t room;
	wchar_t visited;
	wchar_t action;
	float data;
	LinkNode * next = nullptr;
};

