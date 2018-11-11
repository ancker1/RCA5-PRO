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
    this->connectedToLeft.push_back(leftCell);
}

void Cellpoint::setPointRight(Point rightCell)
{
    this->connectedToRight.push_back(rightCell);
}

vector<Point> Cellpoint::getPointLeft()
{
    return this->connectedToLeft;
}

vector<Point> Cellpoint::getPointRight()
{
    return this->connectedToRight;
}

Point Cellpoint::getOnCell()
{
    return this->onCell;
}
