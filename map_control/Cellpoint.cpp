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
    return connectedTo;
}

Point Cellpoint::getOnCell()
{
    return this->onCell;
}

double Cellpoint::getHeuristicdist()
{
    return this->heuristicdist;
}

double Cellpoint::getAstarWeight()
{
    return this->astarWeight;
}

void Cellpoint::calculateHeuristicdist(Point goal)
{
    this->heuristicdist = sqrt(pow(this->onCell.x - goal.x,2) + pow(this->onCell.y - goal.y,2)); // PYTTE
}

void Cellpoint::setAstarWeight(double t)
{
    this->astarWeight = t;
}

Cellpoint::~Cellpoint()
{

}
