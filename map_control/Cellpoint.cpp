#include "Cellpoint.h"

Cellpoint::Cellpoint()
{

}

Cellpoint::Cellpoint(Point onCell)
{
    this->onCell = onCell;
}

void Cellpoint::setPointLeft(Point leftCell)
{
    this->connectedTo[0] = leftCell;
}

void Cellpoint::setPointRight(Point rightCell)
{
    this->connectedTo[1] = rightCell;
}

Point Cellpoint::getPointLeft()
{
    return this->connectedTo[0];
}

Point Cellpoint::getPointRight()
{
    return this->connectedTo[1];
}

Point Cellpoint::getOnCell()
{
    return this->onCell;
}
