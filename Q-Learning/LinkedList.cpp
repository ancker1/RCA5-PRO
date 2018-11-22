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
		std::cout << "Room: " << cur->GetRoom() << ",		Visited: " << cur->GetVisited() << ",		Data: " << cur->GetData() << std::endl;

}

float LinkedList::GetQValue(wchar_t room, wchar_t visited)
{
	for (LinkNode* cur = &head; cur != nullptr; cur = cur->GetNext())
		if (cur->GetRoom() == room && cur->GetVisited() == visited)
			return cur->GetData();
	this->add(new LinkNode(room, visited, 0));	// Create new node if none was found. New node is initialized with: data = 0.
	return 0.0;
}


LinkedList::~LinkedList()
{
}
