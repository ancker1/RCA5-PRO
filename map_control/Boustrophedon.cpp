#include "Boustrophedon.h"

/****************** Contructer/Decontructer *****************/
Boustrophedon::Boustrophedon()
{
}

Boustrophedon::Boustrophedon(Mat map)
{
    Mat temp;
    cvtColor(map, temp, COLOR_BGR2GRAY);
    this->map = temp;
}

Boustrophedon::~Boustrophedon()
{
}

/*********************** Get Functions **********************/
Mat Boustrophedon::getMap()
{
    return this->map;
}

vector<Point> Boustrophedon::getCorners()
{
    return this->corners;
}

vector<Point> Boustrophedon::getUpperMidpoints()
{
    return this->upperMidpoints;
}

vector<Point> Boustrophedon::getLowerMidpoints()
{
    return this->lowerMidpoints;
}

/********************** Print Functions *********************/
Mat Boustrophedon::drawNShowPoints(vector<vector<Point>> points)
{
    Mat tempMap;
    cvtColor(this->map, tempMap, COLOR_GRAY2BGR);
    for(size_t i = 0; i < points.size(); i++)
    {
        for(size_t j = 0; j < points[i].size(); j++)
        {
            tempMap.at<Vec3b>(points[i][j].y,points[i][j].x) = Vec3b(0,0,255); //Drawn points red
        }
    }
    return tempMap;
}

Mat Boustrophedon::drawCellsPath(vector<Cell> cells)
{
    Mat tempMap;
    cvtColor(this->map, tempMap, COLOR_GRAY2BGR);
    // Line options
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    for(size_t i = 0; i < cells.size(); i++)
    {
        for(size_t k = 0; k < cells[i].getAllCellPoints().size(); k++)
        {
            for(size_t j = 0; j < cells[i].getAllCellPoints()[k].getLinks().size(); j++)
            {
                line(tempMap, cells[i].getAllCellPoints()[k].getOnCell(), cells[i].getAllCellPoints()[k].getLinks()[j].getConnectedTo(), Scalar(0,0,255), thickness, lineType, shift);
            }
        }
    }
    return tempMap;
}

/******************* Calculation Functions ******************/
vector<Cell> Boustrophedon::calculateCells()
{
    vector<Cell> detectedCells;
    vector<Point> totalMidpoints;
    cornerDetection();
    findMidPoint();
    cout << this->lowerMidpoints.size() << endl;
    // Initilize Midpoints
    for(size_t i = 0; i < this->upperMidpoints.size(); i++ )
    {
        totalMidpoints.push_back(this->upperMidpoints[i]);
    }
    for(size_t i = 0; i < this->lowerMidpoints.size(); i++ )
    {
        totalMidpoints.push_back(this->lowerMidpoints[i]);
    }

    // Does not need two point with same x right beside eachother
    for(size_t i = 0; i < totalMidpoints.size(); i++)
    {
        for(size_t j = 0; j < totalMidpoints.size(); j++)
        {
            if(totalMidpoints[i].x == totalMidpoints[j].x && totalMidpoints[i].y == totalMidpoints[j].y - 1)
            {
                totalMidpoints.erase(totalMidpoints.begin() + i);
            }
        }
    }

    // Split up in points with none same x points and with same x points without obstacle
    vector<Point> samex = findSamexPointWithoutObstacle(totalMidpoints);
    vector<Point> nonSamex = totalMidpoints;
    // Removes all same x without ostacle from totalMidpoints
    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        for(size_t j = 0; j < samex.size(); j++)
        {
            if(nonSamex[i] == samex[j])
                nonSamex.erase(nonSamex.begin()+i);
        }
    }

    // Calculating Non-samex connecting cells
    Point closest;
    string answer;
    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        Cellpoint tempCellePoint(nonSamex[i]);
        // LEFT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointLeft(samex, nonSamex, tempCellePoint.getOnCell());
        if(answer == "No Point")
        {
            //cout << "Ingen til venstre for i punktet: " << nonSamex[i] << endl;
        }
        else if(answer == "Non Same x")
        {
            Link tempLink(closest,'L');
            tempLink.calculateDijkstra(tempCellePoint.getOnCell());
            tempCellePoint.addConnection(tempLink);
        }
        else if(answer == "Same x")
        {
            for(size_t j = 0; j < samex.size(); j++) // Get all same x points which can be connected to this cellpoint
            {
                if(closest.x == samex[j].x)
                {
                    if(!metObstacleDownOrUp(tempCellePoint.getOnCell(),samex[j]))
                        if(!metObstacleLeft(tempCellePoint.getOnCell(),samex[j]))
                        {
                            Link tempLink(closest,'L');
                            tempLink.calculateDijkstra(tempCellePoint.getOnCell());
                            tempCellePoint.addConnection(tempLink);
                        }
                }
            }
        }

        // RIGHT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointRight(samex, nonSamex, tempCellePoint.getOnCell());
        if(answer == "No Point")
        {
            //cout << "Ingen til højre for i punktet: " << nonSamex[i] << endl;
        }
        else if(answer == "Non Same x")
        {
            Link tempLink(closest,'R');
            tempLink.calculateDijkstra(tempCellePoint.getOnCell());
            tempCellePoint.addConnection(tempLink);
        }
        else if(answer == "Same x")
        {
            for(size_t j = 0; j < samex.size(); j++) // Get all same x points which can be connected to this cellpoint
            {
                if(closest.x == samex[j].x)
                {
                    if(!metObstacleDownOrUp(tempCellePoint.getOnCell(),samex[j]))
                        if(!metObstacleRight(tempCellePoint.getOnCell(),samex[j]))
                        {
                            Link tempLink(closest,'R');
                            tempLink.calculateDijkstra(tempCellePoint.getOnCell());
                            tempCellePoint.addConnection(tempLink);
                        }

                }
            }
        }
        Cell tempCell(tempCellePoint);
        detectedCells.push_back(tempCell);
    }
    // Calculating same x connecting cells
    //Initializing all cellpoint for same x
    vector<Point> tempsamex = samex;
    vector<Point> tempNonSamex = nonSamex;
    vector<Cellpoint> tempCellPointVector;
    for(size_t i = 0; i < samex.size(); i++)
    {
        Cellpoint tempCellePoint(samex[i]);
        // LEFT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointLeft(samex, nonSamex, tempCellePoint.getOnCell());
        if(answer == "No Point")
        {
            //cout << "Ingen til venstre for i punktet: " << samex[i] << endl;
        }
        else if(answer == "Non Same x" || answer == "Same x")
        {
            while(obstacleDetectedWithLine(closest,tempCellePoint.getOnCell()) && answer != "No Point") // stop searching if no point
            {
                findNremovePoint(tempsamex, closest);
                findNremovePoint(tempNonSamex, closest);
                tie(answer,closest) = getClosestPointLeft(tempsamex, tempNonSamex, tempCellePoint.getOnCell());
            }
            if(answer == "Non Same x" || answer == "Same x")
            {
                Link tempLink(closest,'L');
                tempLink.calculateDijkstra(nonSamex[i]);
                tempCellePoint.addConnection(tempLink);
            }

        }

        tempsamex = samex;
        tempNonSamex = nonSamex;
        // RIGHT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointRight(samex, nonSamex, tempCellePoint.getOnCell());
        if(answer == "No Point")
        {
            //cout << "Ingen til højre for i punktet: " << samex[i] << endl;
        }
        else if(answer == "Non Same x" || answer == "Same x")
        {
            while(obstacleDetectedWithLine(closest,tempCellePoint.getOnCell()) && answer != "No Point")
            {
                findNremovePoint(tempsamex, closest);
                findNremovePoint(tempNonSamex, closest);
                tie(answer,closest) = getClosestPointRight(tempsamex, tempNonSamex, tempCellePoint.getOnCell());
            }
            if(answer == "Non Same x" || answer == "Same x")
            {
                Link tempLink(closest,'R');
                tempLink.calculateDijkstra(nonSamex[i]);
                tempCellePoint.addConnection(tempLink);
            }
        }
        tempsamex = samex;
        tempNonSamex = nonSamex;
        tempCellPointVector.push_back(tempCellePoint);
    }

    // Finding same x points which belongs together
    for(size_t i = 0; i < tempCellPointVector.size(); )
    {
        Cell tempCell(tempCellPointVector[i]);
        while(tempCellPointVector[i].getOnCell().x == tempCellPointVector[i+1].getOnCell().x)
        {
            tempCell.addCellPoint(tempCellPointVector[i+1]);
            i++;
        }
        detectedCells.push_back(tempCell);
        i++;
    }

    return detectedCells;
}


/***************** Private Helping Functions ****************/
void Boustrophedon::cornerDetection()
{
    Mat tempMap;
    int filter[2][2] = {{1,3},{5,7}};
    int sum;
    Point corner;
    bitwise_not(this->map,tempMap);
    for(int i = 0; i < tempMap.rows - 1; i++)
    {
        for(int j = 0; j < tempMap.cols - 1; j++)
        {
            sum = 0;
            sum += (tempMap.at<uchar>(i,j)/255) * filter[0][0]; // 255 to get binary
            sum += (tempMap.at<uchar>(i,j+1)/255) * filter[0][1];
            sum += (tempMap.at<uchar>(i+1,j)/255) * filter[1][0];
            sum += (tempMap.at<uchar>(i+1,j+1)/255) * filter[1][1];
            switch (sum) {

            case 1:
                corner.x = j+1;
                corner.y = i+1;
                this->corners.push_back(corner);
                break;

            case 3:
                corner.x = j;
                corner.y = i+1;
                this->corners.push_back(corner);
                break;

            case 5:
                corner.x = j+1;
                corner.y = i;
                this->corners.push_back(corner);
                break;

            case 7:
                corner.x = j;
                corner.y = i;
                this->corners.push_back(corner);
                break;
            /* INDRE PUNKTER BRUGES IKKE PT.
            case 9:
                corner.x = j+1;
                corner.y = i+1;
                this->corners.push_back(corner);
                break;

            case 11:
                corner.x = j;
                corner.y = i+1;
                this->corners.push_back(corner);
                break;

            case 13:
                corner.x = j+1;
                corner.y = i;
                this->corners.push_back(corner);
                break;

            case 15:
                corner.x = j;
                corner.y = i;
                this->corners.push_back(corner);
                break;
           */
            }
        }
    }
}

void Boustrophedon::findMidPoint()
{
    int rowi;
    int colj;
    int Linelength = 0;
    Point midpoint;
    for(size_t i = 0; i < this->corners.size(); i++)
    {
        rowi = this->corners[i].y;
        colj = this->corners[i].x;
        // Not obstacle up and left and up and right
        while((int)this->map.at<uchar>(rowi-1,colj-1) != 0 && (int)this->map.at<uchar>(rowi-1,colj+1) != 0)
        {
            Linelength++;
            rowi -= 1;
        }
        // Point half in line up if line not zero
        if(Linelength != 0)
        {
            midpoint.x = colj;
            midpoint.y = this->corners[i].y-Linelength/2;
            upperMidpoints.push_back(midpoint);
            Linelength = 0;
        }
        rowi = this->corners[i].y;
        colj = this->corners[i].x;
        // Not obstacle down and left and down and right
        while((int)this->map.at<uchar>(rowi+1,colj-1) != 0 && (int)this->map.at<uchar>(rowi+1,colj+1) != 0)
        {
            Linelength++;
            rowi += 1;
        }
        // Point half in line down if line not zero
        if(Linelength != 0)
        {
            midpoint.x = colj;
            midpoint.y = this->corners[i].y+Linelength/2;
            lowerMidpoints.push_back(midpoint);
            Linelength = 0;
        }
    }
}

vector<Point> Boustrophedon::sortxAndRemoveDuplicate(vector<Point> list)
{
    vector<Point> t;
    t.push_back(list[0]);
    int samex = 0;
    for(size_t i = 0; i < list.size(); i++)
    {
        samex = 0;
        for(size_t j = 0; j < t.size(); j++)
        {
            if(list[i] != t[j])
            {
                samex++;
            }
        }
        if(samex == (int)t.size())
            t.push_back(list[i]);
    }

    struct sortx {
      bool operator() (Point point1, Point point2) { return (point1.x < point2.x);}
    } sorting;
    sort(t.begin(), t.end(), sorting);
    return t;
}

vector<Point> Boustrophedon::findSamexPointWithoutObstacle(vector<Point> list)
{
    vector<Point> samexWithoutObs;
    list = sortxAndRemoveDuplicate(list);
    for(size_t i = 0; i < list.size(); i++)
    {
        for(size_t j = 0; j < list.size(); j++)
        {
            if(list[i].x == list[j].x && list[i].y != list[j].y) // If not duplicate but same x
            {
                if(!metObstacleDownOrUp(list[i],list[j])) // No obstacle
                {
                    if(find(samexWithoutObs.begin(), samexWithoutObs.end(), list[i]) == samexWithoutObs.end()) // If not already added
                    {
                        samexWithoutObs.push_back(list[i]);
                    }
                }
            }
        }
    }
    return samexWithoutObs;
}

bool Boustrophedon::metObstacleDownOrUp(Point start, Point end)
{
    if(start.y < end.y)
    {
        for(int i = start.y; i < end.y; i++)
        {
            if((int)this->map.at<uchar>(i,start.x) == 0)
                return true;
        }
    }
    else
    {
        for(int i = start.y; i > end.y; i--)
        {
            if((int)this->map.at<uchar>(i,start.x) == 0)
                return true;
        }
    }
    return false;
}

bool Boustrophedon::metObstacleLeft(Point start, Point end)
{
    end.y = start.y; // Already found out no obstacle up or down
    if(start.x > end.x)
    {
        for(int i = start.x; i > end.x; i--)
        {
            if((int)this->map.at<uchar>(start.y,i) == 0)
            {
                return true;
            }
        }
    }
    else
        return true;
    return false;
}

bool Boustrophedon::metObstacleRight(Point start, Point end)
{
    start.y = end.y; // Already found out no obstacle up or down
    if(start.x < end.x)
    {
        for(int i = start.x; i < end.x; i++)
        {
            if((int)this->map.at<uchar>(start.y,i) == 0)
            {
                return true;
            }
        }
    }
    else
        return true;
    return false;
}

tuple<string, Point> Boustrophedon::getClosestPointLeft(vector<Point> samex, vector<Point> nonSamex, Point cellPoint)
{
    Point closestPointSamex;
    Point closestPointNonSamex;
    closestPointSamex.x = this->map.cols; // NEEDS TO BE INIT ELSE 0
    closestPointNonSamex.x = this->map.cols; // NEEDS TO BE INIT ELSE 0
    for(size_t i = 0; i < samex.size(); i++)
    {
        if(cellPoint != samex[i]) // Closest point is not the same point
        {
            if(cellPoint.y == samex[i].y) // Does not have obstacle op or down if y == same
            {
                if(!metObstacleLeft(cellPoint,samex[i]))
                {
                    if(abs(cellPoint.x - closestPointSamex.x) > abs(cellPoint.x - samex[i].x))
                        closestPointSamex = samex[i];
                }
            }
            else if(!metObstacleDownOrUp(cellPoint,samex[i]))
            {
                if(!metObstacleLeft(cellPoint,samex[i]))
                {
                    if(abs(cellPoint.x - closestPointSamex.x) > abs(cellPoint.x - samex[i].x))
                        closestPointSamex = samex[i];
                }
            }
        }
    }

    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        if(cellPoint != nonSamex[i])
        {
            if(cellPoint.y == nonSamex[i].y) // Does not have obstacle op or down if y == same
            {
                if(!metObstacleLeft(cellPoint,nonSamex[i]))
                {
                    if(abs(cellPoint.x - closestPointNonSamex.x) > abs(cellPoint.x - nonSamex[i].x))
                        closestPointNonSamex = nonSamex[i];
                }
            }
            else if(!metObstacleDownOrUp(cellPoint,nonSamex[i]))
            {
                if(!metObstacleLeft(cellPoint,nonSamex[i]))
                {
                    if(abs(cellPoint.x - closestPointNonSamex.x) > abs(cellPoint.x - nonSamex[i].x))
                        closestPointNonSamex = nonSamex[i];
                }
            }
        }
    }
    if(closestPointNonSamex.x == this->map.cols && closestPointSamex.x == this->map.cols)
        return make_tuple("No Point", Point(0,0));
    if(abs(cellPoint.x - closestPointNonSamex.x) < abs(cellPoint.x - closestPointSamex.x)) // To the power to prevent negative
        return make_tuple("Non Same x",closestPointNonSamex);
    else
        return make_tuple("Same x",closestPointSamex);
}

tuple<string, Point> Boustrophedon::getClosestPointRight(vector<Point> samex, vector<Point> nonSamex, Point cellPoint)
{
    Point closestPointSamex;
    Point closestPointNonSamex;
    closestPointSamex.x = this->map.cols; // NEEDS TO BE INIT ELSE 0
    closestPointNonSamex.x = this->map.cols; // NEEDS TO BE INIT ELSE 0
    for(size_t i = 0; i < samex.size(); i++)
    {
        if(cellPoint != samex[i]) // Closest point is not the same point
        {
            if(cellPoint.y == samex[i].y)// Does not have obstacle op or down if y == same
            {
                if(!metObstacleRight(cellPoint,samex[i]))
                {
                    if(abs(cellPoint.x - closestPointSamex.x) > abs(cellPoint.x - samex[i].x))
                        closestPointSamex = samex[i];
                }
            }
            else if(!metObstacleDownOrUp(cellPoint,samex[i]))
            {
                if(!metObstacleRight(cellPoint,samex[i]))
                {
                    if(abs(cellPoint.x - closestPointSamex.x) > abs(cellPoint.x - samex[i].x))
                        closestPointSamex = samex[i];
                }
            }
        }
    }

    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        if(cellPoint != nonSamex[i])
        {
            if(cellPoint.y == nonSamex[i].y)// Does not have obstacle op or down if y == same
            {
                if(!metObstacleRight(cellPoint,nonSamex[i]))
                {
                    if(abs(cellPoint.x - closestPointNonSamex.x) > abs(cellPoint.x - nonSamex[i].x))
                        closestPointNonSamex = nonSamex[i];
                }
            }
            else if(!metObstacleDownOrUp(cellPoint,nonSamex[i]))
            {
                if(!metObstacleRight(cellPoint,nonSamex[i]))
                {
                    if(abs(cellPoint.x - closestPointNonSamex.x) > abs(cellPoint.x - nonSamex[i].x))
                        closestPointNonSamex = nonSamex[i];
                }
            }
        }
    }
    if(closestPointNonSamex.x == this->map.cols && closestPointSamex.x == this->map.cols)
        return make_tuple("No Point", Point(0,0));
    if(abs(cellPoint.x - closestPointNonSamex.x) < abs(cellPoint.x - closestPointSamex.x)) // To the power to prevent negative
        return make_tuple("Non Same x",closestPointNonSamex);
    else
        return make_tuple("Same x",closestPointSamex);
}

bool Boustrophedon::obstacleDetectedWithLine(Point start, Point end)
{
    LineIterator it(this->map, start, end, 8); // 8 for diagonal
    for (int i = 0; i < it.count; i++, it++) // next point on line
    {
        if ( (int)this->map.at<uchar>( it.pos() ) == 0 )
            return true;
    }
    return false;
}

bool Boustrophedon::findNremovePoint(vector<Point> &list, Point point)
{
    for(size_t i = 0; i < list.size(); i++)
    {
        if(list[i] == point)
        {
            list.erase(list.begin()+i);
            return true;
        }
    }
    return false;
}
