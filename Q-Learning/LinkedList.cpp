#include "LinkedList.h"



LinkedList::LinkedList(LinkNode head)
{
	this->head = head;
	size = 0;
}

void LinkedList::add(LinkNode* node)
{
	LinkNode* next;
	LinkNode* prev;
	LinkNode* cur;
	cur = &head;
	while (cur->GetNext() != nullptr)
	{
		cur = cur->GetNext();
	}
	cur->SetNext(node);
	size++;
}

void LinkedList::print_data()
{
	for (LinkNode* cur = &head; cur != nullptr; cur = cur->GetNext())
		std::cout << "Room: " << cur->GetRoom() << ",		Visited: " << cur->GetVisited() << ",		Action: " << cur->GetAction()<< ",		Data: " << cur->GetData() << std::endl;

}

float LinkedList::GetValue(wchar_t room, wchar_t visited, wchar_t action)
{
	for (LinkNode* cur = &head; cur != nullptr; cur = cur->GetNext())
		if (cur->GetRoom() == room && cur->GetVisited() == visited && cur->GetAction() == action)
			return cur->GetData();
	this->add(new LinkNode(room, visited, action, 0.0));	// Create new node if none was found. New node is initialized with: data = 0.
	return 0.0;
}

void LinkedList::SetValue(wchar_t room, wchar_t visited, wchar_t action, float data)
{
	for (LinkNode* cur = &head; cur != nullptr; cur = cur->GetNext())
		if (cur->GetRoom() == room && cur->GetVisited() == visited && cur->GetAction() == action)
		{
			cur->SetData(data);
			return;
		}
	this->add(new LinkNode(room, visited, action, data));	// Create new node if none was found. New node is initialized with: data = data.
}

int LinkedList::GetSize()
{
	return size;
}


LinkedList::~LinkedList()
{
}
