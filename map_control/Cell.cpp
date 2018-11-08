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

Point Cell::getCellPointPointLeft()
{
    return this->cellpoint.getPointLeft();
}

Point Cell::getCellPointPointRight()
{
    return this->cellpoint.getPointRight();
}
