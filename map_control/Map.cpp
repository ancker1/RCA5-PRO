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

void Map::printMap()
{
    Mat resizeMap;
    resize(map,resizeMap,map.size()*10,0,0,INTER_NEAREST);
    imshow("Map", resizeMap);
}

vector<Point> Map::cornerDetection()
{
    vector<Point> vecCorners;
    int filter[2][2] = {{5,7},{9,11}};
    int sum;
    Point corner;
    bitwise_not(map,map);
    for(int i = 0; i < map.rows-1; i++)
    {
        for(int j = 0; j < map.cols-1; j++)
        {
            sum = 0;
            sum += (map.at<uchar>(i,j)/255) * filter[0][0]; //   /255 to get binary
            sum += (map.at<uchar>(i,j+1)/255) * filter[0][1];
            sum += (map.at<uchar>(i+1,j)/255) * filter[1][0];
            sum += (map.at<uchar>(i+1,j+1)/255) * filter[1][1];
            switch (sum) {

            case 5:
                corner.x = j+1;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 7:
                corner.x = j;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 9:
                corner.x = j+1;
                corner.y = i;
                vecCorners.push_back(corner);
                break;

            case 11:
                corner.x = j;
                corner.y = i;
                vecCorners.push_back(corner);
                break;
            /* INDRE PUNKTER BRUGES IKKE PT.
            case 21:
                corner.x = j+1;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 23:
                corner.x = j;
                corner.y = i+1;
                vecCorners.push_back(corner);
                break;

            case 25:
                corner.x = j+1;
                corner.y = i;
                vecCorners.push_back(corner);
                break;

            case 27:
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
    for(int i = 0; i < criticalPoints.size(); i++)
    {
        rowi = criticalPoints[i].y;
        colj = criticalPoints[i].x;
        // Line up

        while(map.at<uchar>(rowi-1,colj) != 255)
        {
            Linelength++;
            rowi -= 1;
        }

        // Point half in line up
        goal.x = colj;
        goal.y = criticalPoints[i].y-Linelength/2;
        upperTrapezoidalGoals.push_back(goal);
        Linelength = 0;

        // Line down
        rowi = criticalPoints[i].y;
        colj = criticalPoints[i].x;
        while(map.at<uchar>(rowi+1,colj) != 255)
        {
            Linelength++;
            rowi += 1;
        }
        // Point half in line down
        goal.x = colj;
        goal.y = criticalPoints[i].y+Linelength/2;
        lowerTrapezoidalGoals.push_back(goal);
        Linelength = 0;

    }
}

void Map::drawNShowPoints(string pictureText, vector<Point> points)
{
    Mat tempMap;
    cvtColor(map, tempMap, COLOR_GRAY2BGR);
    for(int i = 0; i < points.size(); i++)
    {
        tempMap.at<Vec3b>(points[i].y,points[i].x)[1] = 255;
    }
    resize(tempMap,tempMap,map.size()*10,0,0,INTER_NEAREST);
    imshow(pictureText, tempMap);


}

vector<Point> Map::getUpperTrapezoidalGoals()
{
    return upperTrapezoidalGoals;
}

vector<Point> Map::getLowerTrapezoidalGoals()
{
    return lowerTrapezoidalGoals;
}

Map::~Map()
{

}

