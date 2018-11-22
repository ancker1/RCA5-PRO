#pragma once
#include "LinkNode.h"
#include <iostream>
#include <string>
using namespace std;
class LinkedList
{
public:
	LinkedList(LinkNode head);
	void add(LinkNode * node);
	void print_data();
	float GetValue(wchar_t room, wchar_t visited, wchar_t action);// If no one on list is contains the combo of room and visited, then create one.
	void SetValue(wchar_t room, wchar_t visited, wchar_t action, float data);
	int GetSize();
	~LinkedList();
private:
	LinkNode head;
	int size;
};

