#include "Cellpoint.h"

Cellpoint::Cellpoint()
{

}

Cellpoint::Cellpoint(Point onCell)
{
    this->onCell = onCell;
}

void Cellpoint::addConnection(Link connection)
{
    connectedTo.push_back(connection);
}

vector<Link> Cellpoint::getLinks()
{
    return this->connectedTo;
}

void Cellpoint::removePointFromLinks(Point p)
{
    for(size_t i = 0; i < this->connectedTo.size(); i++)
    {
        if(this->connectedTo[i].getConnectedTo() == p)
            this->connectedTo.erase(this->connectedTo.begin()+i);
    }
}

Point Cellpoint::getOnCell()
{
    return this->onCell;
}

double Cellpoint::getHeuristicdist()
{
    return this->heuristicdist;
}

double Cellpoint::getCombinedHeuristic()
{
    return this->combinedHeuristic;
}

void Cellpoint::calculateHeuristicdist(Point goal)
{
    this->heuristicdist = sqrt(pow(this->onCell.x - goal.x,2) + pow(this->onCell.y - goal.y,2)); // PYTTE
}

void Cellpoint::setCombinedHeuristic(double t)
{
    this->combinedHeuristic = t;
}

Cellpoint::~Cellpoint()
{

}
