#include "LinkNode.h"



LinkNode::LinkNode()
{
}

LinkNode::LinkNode(wchar_t room, wchar_t visited, wchar_t action, float data)
{
	this->data = data;
	this->room = room;
	this->visited = visited;
	this->action = action;
}

void LinkNode::SetNext(LinkNode * Node)
{
	next = Node;
}

LinkNode* LinkNode::GetNext()
{
	return next;
}

float LinkNode::GetData()
{
	return data;
}

wchar_t LinkNode::GetRoom()
{
	return room;
}

wchar_t LinkNode::GetVisited()
{
	return visited;
}

wchar_t LinkNode::GetAction()
{
	return action;
}

void LinkNode::SetData(float d)
{
	data = d;
}


LinkNode::~LinkNode()
{
}
