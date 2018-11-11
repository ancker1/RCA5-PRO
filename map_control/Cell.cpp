#include "Cell.h"

Cell::Cell()
{

}

Cell::Cell(Cellpoint a)
{
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
