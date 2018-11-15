#include "Cell.h"

Cell::Cell()
{

}

Cell::Cell(Cellpoint a)
{
    this->allCellPoints.push_back(a);
    this->cellpoint = a;
}

Point Cell::getCellPointOnCell()
{
    return this->cellpoint.getOnCell();
}

vector<Point> Cell::getCellPointLeft()
{
    return this->cellpoint.getPointLeft();
}

vector<Point> Cell::getCellPointRight()
{
    return this->cellpoint.getPointRight();
}

vector<Cellpoint> Cell::getAllCellPoints()
{
    return this->allCellPoints;
}

void Cell::addCellPoint(Cellpoint newCellPoint)
{
    this->allCellPoints.push_back(newCellPoint);
}


Cell::~Cell(){}
