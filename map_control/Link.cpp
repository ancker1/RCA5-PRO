#include "Link.h"

Link::Link()
{

}

Link::Link(Point connectedTo, char leftOrRight)
{
    this->connectedTo = connectedTo;
    this->leftOrRight = leftOrRight;
}

Point Link::getConnectedTo()
{
    return this->connectedTo;
}

char Link::getLeftOrRight()
{
    return this->leftOrRight;
}

double Link::getDijkstraDist()
{
    return this->dijkstraDist;
}

void Link::calculateDijkstra(Point cellPoint)
{
    this->dijkstraDist = sqrt(pow(cellPoint.x - this->connectedTo.x, 2) + pow(cellPoint.y - this->connectedTo.y, 2));
}

Link::~Link(){}
