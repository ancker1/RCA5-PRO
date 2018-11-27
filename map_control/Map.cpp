#include "Map.h"

Map::Map()
{

}

Map::Map(Mat picture)
{
    map = picture;
}

int Map::getMapRows()
{
    return map.rows;
}

int Map::getMapCols()
{
    return map.cols;
}

Mat Map::getSweepLineMap()
{
    return sweepLineMap;
}
void Map::printMap()
{
    Mat resizeMap;
    resize(map,resizeMap,map.size()*10,0,0,INTER_NEAREST);
    imshow("Map", resizeMap);
}

void Map::print_map(Mat &map, string s) {
    Mat resizeMap;
    resize( map, resizeMap, map.size()*10, 0, 0, INTER_NEAREST);
    imshow(s, resizeMap);
}

vector<Point> Map::cornerDetection()
{
    vector<Point> vecCorners;
    int filter[2][2] = {{1,3},{5,7}};
    int sum;
    Point corner;
    bitwise_not(map,map);
    for(int i = 0; i < map.rows-1; i++)
    {
        for(int j = 0; j < map.cols-1; j++)
        {
            sum = 0;
            sum += (map.at<uchar>(i,j)/255) * filter[0][0]; // 255 to get binary
            sum += (map.at<uchar>(i,j+1)/255) * filter[0][1];
            sum += (map.at<uchar>(i+1,j)/255) * filter[1][0];
            sum += (map.at<uchar>(i+1,j+1)/255) * filter[1][1];
            switch (sum) {

            case 1:
                corner.x = j+1;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 3:
                corner.x = j;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 5:
                corner.x = j+1;
                corner.y = i;
                vecCorners.push_back(corner);
                break;

            case 7:
                corner.x = j;
                corner.y = i;
                vecCorners.push_back(corner);
                break;
            /* INDRE PUNKTER BRUGES IKKE PT.
            case 9:
                corner.x = j+1;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 11:
                corner.x = j;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 13:
                corner.x = j+1;
                corner.y = i;
                vecCorners.push_back(corner);
                break;

            case 15:
                corner.x = j;
                corner.y = i;
                vecCorners.push_back(corner);
                break;
           */
            }
        }
    }
    /* DRAWS CORNERS USE INSTEAD FUNCTION
    cvtColor(map, map, COLOR_GRAY2BGR);
    for(int i = 0; i < vecCorners.size(); i++)
    {
        int rowi = vecCorners[i].y;
        int colj = vecCorners[i].x;
        map.at<Vec3b>(rowi, colj)[0] = 255;
        cout << "x: " << colj << " y: " << rowi << endl;
    }
    */
    return vecCorners;
}

void Map::trapezoidalLines(vector<Point> criticalPoints)
{
    int rowi;
    int colj;
    int Linelength = 0;
    Point goal;
    for(size_t i = 0; i < criticalPoints.size(); i++)
    {
        rowi = criticalPoints[i].y;
        colj = criticalPoints[i].x;
        // Not obstacle up and left and up and right
        while(map.at<uchar>(rowi-1,colj-1) != 255 && map.at<uchar>(rowi-1,colj+1) != 255)
        {

            Linelength++;
            rowi -= 1;
        }
        // Point half in line up if line not zero
        if(Linelength != 0)
        {
            goal.x = colj;
            goal.y = criticalPoints[i].y-Linelength/2;
            upperTrapezoidalGoals.push_back(goal);
            Linelength = 0;
        }
        rowi = criticalPoints[i].y;
        colj = criticalPoints[i].x;
        // Not obstacle down and left and down and right
        while(map.at<uchar>(rowi+1,colj-1) != 255 && map.at<uchar>(rowi+1,colj+1) != 255)
        {
            Linelength++;
            rowi += 1;
        }
        // Point half in line down if line not zero
        if(Linelength != 0)
        {
            goal.x = colj;
            goal.y = criticalPoints[i].y+Linelength/2;
            lowerTrapezoidalGoals.push_back(goal);
            Linelength = 0;
        }
    }
}

void Map::drawNShowPoints(string pictureText, vector<Point> points)
{
    Mat tempMap;
    cvtColor(map, tempMap, COLOR_GRAY2BGR);
    for(size_t i = 0; i < points.size(); i++)
    {
        tempMap.at<Vec3b>(points[i].y,points[i].x)[1] = 255;
    }
    resize(tempMap,tempMap,map.size()*10,0,0,INTER_NEAREST);
    imshow(pictureText, tempMap);
}

void Map::drawCellsPath(string pictureText, vector<Cell> cells)
{
    Mat tempMap;
    // Line options
    int thickness = 1;
    int lineType = 8;
    int shift = 0;
    int scaling = 10;
    cvtColor(map, tempMap, COLOR_GRAY2BGR);
    bitwise_not(tempMap,tempMap);
    //resize(tempMap,tempMap,map.size()*scaling,0,0,INTER_NEAREST); // Udkommenter for små streger
    for(size_t i = 0; i < cells.size(); i++)
    {
        for(size_t k = 0; k < cells[i].getAllCellPoints().size(); k++)
        {
            for(size_t j = 0; j < cells[i].getAllCellPoints()[k].getLinks().size(); j++)
            {
                //line(tempMap, cells[i].getAllCellPoints()[k].getOnCell()*scaling, cells[i].getAllCellPoints()[k].getLinks()[j].getConnectedTo()*scaling, Scalar(0,0,255), thickness, lineType, shift); // Udkommenter for små streger
                line(tempMap, cells[i].getAllCellPoints()[k].getOnCell(), cells[i].getAllCellPoints()[k].getLinks()[j].getConnectedTo(), Scalar(0,0,255), thickness, lineType, shift);
            }
             //waitKey(0);
             // Udkommenter hvis følg med imshow(pictureText, tempMap);
        }
    }
    resize(tempMap,tempMap,map.size()*scaling,0,0,INTER_NEAREST);
    imshow(pictureText, tempMap);

}

vector<Cell> Map::calculateCells(vector<Point> upperTrap, vector<Point> lowerTrap)
{
    vector<Cell> detectedCells;
    vector<Point> totalTrapGoals;
    int rowi = 0;
    int colj = 0;
    cvtColor(map, sweepLineMap, COLOR_GRAY2BGR);
    // INIT SUBGOALS
    for(size_t i = 0; i < upperTrap.size(); i++ )
    {
        totalTrapGoals.push_back(upperTrap[i]);
    }
    for(size_t i = 0; i < lowerTrap.size(); i++ )
    {
        totalTrapGoals.push_back(lowerTrap[i]);
    }
    // Draws sweeplines
    for(size_t i = 0; i < totalTrapGoals.size(); i++)
    {
        rowi = totalTrapGoals[i].y;
        colj = totalTrapGoals[i].x;
        // Not obstacle up
        while(map.at<uchar>(rowi,colj) != 255)
        {
            sweepLineMap.at<Vec3b>(rowi,colj)[0] = 255;
            rowi -= 1;
        }
        rowi = totalTrapGoals[i].y;
        // Not obstacle down
        while(map.at<uchar>(rowi,colj) != 255)
        {
            sweepLineMap.at<Vec3b>(rowi,colj)[0] = 255;
            rowi += 1;
        }
    }
    // Does not need two point with same x right beside eachother
    for(size_t i = 0; i < totalTrapGoals.size(); i++)
    {
        for(size_t j = 0; j < totalTrapGoals.size(); j++)
        {
            if(totalTrapGoals[i].x == totalTrapGoals[j].x && totalTrapGoals[i].y == totalTrapGoals[j].y - 1)
            {
                totalTrapGoals.erase(totalTrapGoals.begin() + i);
            }
        }
    }


    totalTrapGoals = sortxAndRemoveDuplicate(totalTrapGoals);
    vector<Point> samex;
    vector<Point> nonSamex = totalTrapGoals;
    samex = findSamexPointWithoutObstacle(totalTrapGoals);
    // Removes all same x without ostacle from totalTrapGoals
    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        for(size_t j = 0; j < samex.size(); j++)
        {
            if(nonSamex[i] == samex[j])
                nonSamex.erase(nonSamex.begin()+i);
        }
    }

    //drawNShowPoints("No same x", nonSamex);
    //drawNShowPoints("Same x", samex);

    Point closest;
    string answer;
    vector<Cellpoint> allCellPoints;
    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        allCellPoints.push_back(nonSamex[i]);
    }
    //Non-samex connecting cells
    for(size_t i = 0; i < nonSamex.size(); i++)
    {
        Cellpoint tempCellePoint(nonSamex[i]);
        // LEFT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointLeft(samex, nonSamex, nonSamex[i]);
        if(answer == "No Point")
        {
            cout << "Ingen til venstre for i punktet: " << nonSamex[i] << endl;
        }
        else if(answer == "Non Same x")
        {
            Link tempLink(closest,'L');
            tempLink.calculateDijkstra(nonSamex[i]);
            tempCellePoint.addConnection(tempLink);
        }
        else if(answer == "Same x")
        {
            for(size_t j = 0; j < samex.size(); j++)
            {
                if(closest.x == samex[j].x)
                {
                    if(!metObstacleDownOrUp(tempCellePoint.getOnCell(),samex[j]))
                        if(!metObstacleLeft(tempCellePoint.getOnCell(),samex[j]))
                        {
                            Link tempLink(closest,'L');
                            tempLink.calculateDijkstra(nonSamex[i]);
                            tempCellePoint.addConnection(tempLink);
                        }
                }
            }
        }

        // RIGHT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointRight(samex, nonSamex, nonSamex[i]);
        if(answer == "No Point")
        {
            cout << "Ingen til højre for i punktet: " << nonSamex[i] << endl;
        }
        else if(answer == "Non Same x")
        {
            Link tempLink(closest,'R');
            tempLink.calculateDijkstra(nonSamex[i]);
            tempCellePoint.addConnection(tempLink);
        }
        else if(answer == "Same x")
        {
            for(size_t j = 0; j < samex.size(); j++)
            {
                if(closest.x == samex[j].x)
                {
                    if(!metObstacleDownOrUp(tempCellePoint.getOnCell(),samex[j]))
                        if(!metObstacleRight(tempCellePoint.getOnCell(),samex[j]))
                        {
                            Link tempLink(closest,'R');
                            tempLink.calculateDijkstra(nonSamex[i]);
                            tempCellePoint.addConnection(tempLink);
                        }

                }
            }
        }
        Cell tempCell(tempCellePoint);
        detectedCells.push_back(tempCell);
    }

    //Initializing all cellpoint for same x
    vector<Point> tempsamex = samex;
    vector<Point> tempNonSamex = nonSamex;
    vector<Cellpoint> tempCellPointVector;
    for(size_t i = 0; i < samex.size(); i++)
    {
        Cellpoint tempCellePoint(samex[i]);
        // LEFT FOR CELLEPOINT
        tie(answer,closest) = getClosestPointLeft(samex, nonSamex, samex[i]);
        if(answer == "No Point")
        {
            cout << "Ingen til venstre for i punktet: " << samex[i] << endl;
        }
        else if(answer == "Non Same x" || answer == "Same x")
        {
            while(obstacleDetectedWithLine(closest,samex[i]) && answer != "No Point")
            {
                findNremovePoint(tempsamex, closest);
                findNremovePoint(tempNonSamex, closest);
                tie(answer,closest) = getClosestPointLeft(tempsamex, tempNonSamex, samex[i]);
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
        tie(answer,closest) = getClosestPointRight(samex, nonSamex, samex[i]);
        if(answer == "No Point")
        {
            cout << "Ingen til højre for i punktet: " << samex[i] << endl;
        }
        else if(answer == "Non Same x" || answer == "Same x")
        {
            while(obstacleDetectedWithLine(closest,samex[i]) && answer != "No Point")
            {
                findNremovePoint(tempsamex, closest);
                findNremovePoint(tempNonSamex, closest);
                tie(answer,closest) = getClosestPointRight(tempsamex, tempNonSamex, samex[i]);
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

    //Samex connecting cells
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


    drawCellsPath("t",detectedCells);
    return detectedCells;
}

vector<Point> Map::getUpperTrapezoidalGoals()
{
    return upperTrapezoidalGoals;
}

vector<Point> Map::getLowerTrapezoidalGoals()
{
    return lowerTrapezoidalGoals;
}

vector<Point_<double>> Map::convertToGazeboCoordinates(vector<Point> goals)
{
    double widthScale = 20.0/14.0;
    double heightScale = 15.0/11.0;
    double offsetx = 6.5;
    double offsety = 5.5;
    vector<Point_<double>> convertedGoals;
    Point_<double> convertedPoint;

    for(size_t i = 0; i < goals.size(); i++)
    {
        convertedPoint.x = ((double)goals[i].x/widthScale)-offsetx;
        convertedPoint.y = ((double)goals[i].y/heightScale)-offsety;
        convertedGoals.push_back(convertedPoint);
    }
    return convertedGoals;
}

vector<Point_<double>> Map::convertToGazeboCoordinatesTrapezoidal(vector<Point> upperGoals, vector<Point> lowerGoals)
{
    vector<Point_<double>> convertedGoals;
    vector<Point> tempGoals;
    double widthScale = 20.0/14.0;
    double heightScale = 15.0/11.0;
    double offsetx = 6.5;
    double offsety = 4.5;
    Point_<double> convertedPoint;
    vector<Point> deletes;

    for(size_t i = 0; i < upperGoals.size(); i++)
    {
        tempGoals.push_back(upperGoals[i]);
    }
    for(size_t i = 0; i < lowerGoals.size(); i++)
    {
        tempGoals.push_back(lowerGoals[i]);
    }

    // x-coordinates beside eachother y-coordinates beside eachother
    for(size_t i = 0; i < tempGoals.size(); i++)
    {
        for(size_t j = 0; j < tempGoals.size(); j++)
        {
            if((tempGoals[i].x == tempGoals[j].x) && tempGoals[i].y == tempGoals[j].y+1 )
            {
                deletes.push_back(tempGoals[i]);
                deletes.push_back(tempGoals[j]);
                convertedPoint.x = ((double)tempGoals[i].x/widthScale)-offsetx;
                convertedPoint.y = (((double)tempGoals[i].y/heightScale)-offsety) - (((double)tempGoals[i].y/heightScale)-offsety) - (((double)tempGoals[j].y/heightScale)-offsety);
                convertedGoals.push_back((convertedPoint));
            }
            else if((tempGoals[i].y == tempGoals[j].y) && tempGoals[i].x == tempGoals[j].x+1 )
            {
                deletes.push_back(tempGoals[i]);
                deletes.push_back(tempGoals[j]);
                convertedPoint.x = (((double)tempGoals[i].x/widthScale)-offsetx) - (((double)tempGoals[i].x/widthScale)-offsetx) - (((double)tempGoals[j].x/widthScale)-offsetx);
                convertedPoint.y = ((double)tempGoals[i].y/heightScale)-offsety;
                convertedGoals.push_back((convertedPoint));
            }
        }
    }
    // Removes the goals which has same x-coordiantes or y coordinates beside eachother
    for(size_t i = 0; i < deletes.size(); i++)
    {
        for(size_t j = 0; j < tempGoals.size(); j++)
        {
            if(deletes[i] == tempGoals[j])
            {
                tempGoals.erase(tempGoals.begin()+j);
            }
        }
    }
    // Adds the rest of the goals
    for(size_t i = 0; i < tempGoals.size(); i++)
    {
        convertedPoint.x = ((double)tempGoals[i].x/widthScale)-offsetx;
        convertedPoint.y = ((double)tempGoals[i].y/heightScale)-offsety;
        convertedGoals.push_back(convertedPoint);
    }
    return convertedGoals;
}

vector<Point> Map::sortxAndRemoveDuplicate(vector<Point> list)
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
        if(samex == t.size())
            t.push_back(list[i]);
    }

    struct sortx {
      bool operator() (Point point1, Point point2) { return (point1.x < point2.x);}
    } sorting;
    sort(t.begin(), t.end(), sorting);
    return t;
}

vector<Point> Map::findSamexPointWithoutObstacle(vector<Point> list)
{
    vector<Point> samexWithoutObs;
    for(size_t i = 0; i < list.size(); i++)
    {
        for(size_t j = 0; j < list.size(); j++)
        {
            if(list[i].x == list[j].x && list[i].y != list[j].y)
            {
                if(!metObstacleDownOrUp(list[i],list[j]))
                {
                    if(find(samexWithoutObs.begin(), samexWithoutObs.end(), list[i]) == samexWithoutObs.end())
                    {
                        samexWithoutObs.push_back(list[i]);
                    }
                }
            }
        }
    }
    return samexWithoutObs;
}

bool Map::metObstacleDownOrUp(Point start, Point end)
{

    if(start.y < end.y)
    {
        for(int i = start.y; i < end.y; i++)
        {
            if((int)map.at<uchar>(i,start.x) == 255)
                return true;
        }
    }
    else
    {
        for(int i = start.y; i > end.y; i--)
        {
            if((int)map.at<uchar>(i,start.x) == 255)
                return true;
        }
    }
    return false;
}

bool Map::metObstacleLeft(Point start, Point end)
{
    end.y = start.y;
    if(start.x > end.x)
    {
        for(int i = start.x; i > end.x; i--)
        {
            if((int)map.at<uchar>(start.y,i) == 255)
            {
                return true;
            }
        }
    }
    else
        return true;
    return false;
}

bool Map::metObstacleRight(Point start, Point end)
{
    start.y = end.y;
    if(start.x < end.x)
    {
        for(int i = start.x; i < end.x; i++)
        {
            if((int)map.at<uchar>(start.y,i) == 255)
            {
                return true;
            }
        }
    }
    else
        return true;
    return false;
}

tuple<string, Point> Map::getClosestPointLeft(vector<Point> samex, vector<Point> nonSamex, Point cellPoint)
{
    Point closestPointSamex;
    Point closestPointNonSamex;
    closestPointSamex.x = map.cols; // NEEDS TO BE INIT ELSE 0
    closestPointNonSamex.x = map.cols; // NEEDS TO BE INIT ELSE 0
    for(size_t i = 0; i < samex.size(); i++)
    {
        if(!(cellPoint == samex[i]))
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
        if(!(cellPoint == nonSamex[i]))
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
    if(closestPointNonSamex.x == map.cols && closestPointSamex.x == map.cols)
        return make_tuple("No Point", Point(0,0));
    if(abs(cellPoint.x - closestPointNonSamex.x) < abs(cellPoint.x - closestPointSamex.x)) // To the power to prevent negative
        return make_tuple("Non Same x",closestPointNonSamex);
    else
        return make_tuple("Same x",closestPointSamex);
}

tuple<string,Point> Map::getClosestPointRight(vector<Point> samex, vector<Point> nonSamex, Point cellPoint)
{
    Point closestPointSamex;
    Point closestPointNonSamex;
    closestPointSamex.x = map.cols; // NEEDS TO BE INIT ELSE 0
    closestPointNonSamex.x = map.cols; // NEEDS TO BE INIT ELSE 0
    for(size_t i = 0; i < samex.size(); i++)
    {
        if(!(cellPoint == samex[i]))
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
        if(!(cellPoint == nonSamex[i]))
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
    if(closestPointNonSamex.x == map.cols && closestPointSamex.x == map.cols)
        return make_tuple("No Point", Point(0,0));
    if(abs(cellPoint.x - closestPointNonSamex.x) < abs(cellPoint.x - closestPointSamex.x)) // To the power to prevent negative
        return make_tuple("Non Same x",closestPointNonSamex);
    else
        return make_tuple("Same x",closestPointSamex);
}

vector<Point> Map::get_points(LineIterator &it)
{
    vector<Point> v(it.count);
    for (int i = 0; i < it.count; i++, it++) {
        v[i] = it.pos();
    }
    return v;
}

bool Map::obstacleDetectedWithLine(Point start, Point end)
{
    LineIterator it(this->map, start, end, 8);
    vector<Point> v = get_points(it);
    for (size_t i = 0; i < v.size(); i++) {
        if ( (int)this->map.at<uchar>(v[i]) == 255 ) {
            return true;
        }
    }
    return false;
}

bool Map::findNremovePoint(vector<Point> &list, Point point)
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

Map::~Map()
{

}

/****************************************************
 *  BUSHFIRE
 * *************************************************/

Mat Map::brushfire_img(Mat &img) {
    Mat binary_img = img.clone();
    binarize_img(binary_img);
    make_brushfire_grid(binary_img);
    return binary_img;
}

vector<Point> Map::find_centers(Mat &img) {
    Mat1b kernel_lm( Size(5,5), 1u);
    Mat image_dilate;
    dilate(img, image_dilate, kernel_lm);
    Mat1b local_max = (img >= image_dilate);
    vector<Point> v;
    for (int y = 0; y < local_max.rows; y++) {
        for (int x = 0; x < local_max.cols; x++) {
            if ((int)local_max.at<uchar>(y,x) == 255) {
                for (int j = 1; (int)local_max.at<uchar>(y+j,x) == 255; j++) {
                    local_max.at<uchar>(y,x) = 0;
                    j++;
                }
                for (int j = 1; (int)local_max.at<uchar>(y,x+j) == 255; j++) {
                    local_max.at<uchar>(y,x) = 0;
                    j++;
                }
            }
        }
    }

    for (int y = 0; y < local_max.rows; y++) {
        for (int x = 0; x < local_max.cols; x++) {
            if ((int)local_max.at<uchar>(y,x) == 255) {
                Point p(x,y);
                v.push_back(p);
            }
        }
    }

    remove_points_in_corners(v, img);

    return v;
}

void Map::binarize_img(Mat &img) {
    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);
    threshold(gray, img, 128.0, 255.0, THRESH_BINARY);
    img = img > 128;
}

void Map::find_neighbors(vector<Point> &v, Mat &img, int x, int y) {
    Point p;
    if ((int)img.at<uchar>(x-1,y-1)==255) {
        p.x=x-1;
        p.y=y-1;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x,y-1)==255) {
        p.x=x;
        p.y=y-1;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x+1,y-1)==255) {
        p.x=x+1;
        p.y=y-1;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x-1,y)==255) {
        p.x=x-1;
        p.y=y;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x+1,y)==255) {
        p.x=x+1;
        p.y=y;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x-1,y+1)==255) {
        p.x=x-1;
        p.y=y+1;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x,y+1)==255) {
        p.x=x;
        p.y=y+1;
        v.push_back(p);
    }
    if ((int)img.at<uchar>(x+1,y+1)==255) {
        p.x=x+1;
        p.y=y+1;
        v.push_back(p);
    }
}

void Map::make_brushfire_grid(Mat &img) {
    vector<Point> neighbors;
    for (int y = 0; y < img.cols; y++) {
        for (int x = 0; x < img.rows; x++) {
            if ((int)img.at<uchar>(x,y) == 0) {
                find_neighbors(neighbors, img, x, y);
            }
        }
    }

    int color = 1;
    while (!neighbors.empty()) {
        vector<Point> new_neighbors;
        for (size_t i = 0; i < neighbors.size(); i++) {
            if (img.at<uchar>(neighbors[i].x,neighbors[i].y)==255) {
                find_neighbors(new_neighbors, img, neighbors[i].x, neighbors[i].y);
                img.at<uchar>(neighbors[i].x,neighbors[i].y) = color;
            }
        }
        color++;
        neighbors = new_neighbors;
    }
}

void Map::remove_points_in_corners(vector<Point> &v, Mat &img) {
    for (size_t i = 0; i < v.size(); i++) {
        if ((int)img.at<uchar>(v[i].y-1,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y+1,v[i].x-1)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y-1,v[i].x)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y+1,v[i].x)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y-1,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
        if ((int)img.at<uchar>(v[i].y+1,v[i].x+1)==0) {
            v.erase(v.begin()+i);
        }
    }
}

vector<Point> Map::astar(vector<Cellpoint> cellpoints, Point startCellPoint, Point goalCellPoint) // Start and Goal Cells need to be on the Cell list
{

    vector<Point> path;
    vector<Cellpoint> tempCellspoint;
    vector<Cellpoint> aStarPath;

    aStarPath.push_back(findCellPointFromPoint(cellpoints, startCellPoint)); // Starting Cellpoint
    //path.push_back(startCellPoint); // Starting Point

    //Init heauristic with distance from point to goal
    for(size_t i = 0; i < cellpoints.size(); i++)
    {
        cellpoints[i].calculateHeuristicdist(goalCellPoint);
    }

    Cellpoint tempCellpoint ;
    // Sidder Fast Her

    return path;
}

Cellpoint Map::findCellPointFromPoint(vector<Cellpoint> cellpoints, Point point)
{
    for(size_t i = 0; i < cellpoints.size(); i++)
    {
        if(cellpoints[i].getOnCell() == point)
            return cellpoints[i];
    }
}

Cellpoint Map::findSmallestCombinedHeuristic(vector<Cellpoint> cellpoints)
{
    Cellpoint tempCellp = cellpoints[0];
    for(size_t i = 0; i < cellpoints.size(); i++)
    {
        if(tempCellp.getHeuristicdist() > cellpoints[i].getCombinedHeuristic())
            tempCellp = cellpoints[i];
    }
    return tempCellp;
}

Cell Map::findClosestCellFromStart(vector<Cell> cells, Point start, int &cellNumber) // MAKE IT WORK
{
    Cell closestCell = cells[0].getAllCellPoints()[0];
    int closestDist = abs(sqrt(pow(cells[0].getAllCellPoints()[0].getOnCell().x,2)+pow(cells[0].getAllCellPoints()[0].getOnCell().y,2))-sqrt(pow(start.x,2)+pow(start.y,2)));
    for(size_t i = 0; i < cells.size(); i++)
    {
        for(size_t j = 0; j < cells[i].getAllCellPoints().size(); j++)
        {
            if(!obstacleDetectedWithLine(start, cells[i].getAllCellPoints()[j].getOnCell()))
            {
                if(closestDist > abs(sqrt(pow(cells[i].getAllCellPoints()[j].getOnCell().x,2)+pow(cells[i].getAllCellPoints()[j].getOnCell().y,2))-sqrt(pow(start.x,2)+pow(start.y,2))))
                {
                    closestCell = cells[i].getAllCellPoints()[j];
                    closestDist = abs(sqrt(pow(cells[i].getAllCellPoints()[j].getOnCell().x,2)+pow(cells[i].getAllCellPoints()[j].getOnCell().y,2))-sqrt(pow(start.x,2)+pow(start.y,2)));
                    cellNumber = j;
                }
            }
        }
    }
    return closestCell;
}
