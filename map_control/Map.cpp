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
    imshow("Map", map);
    waitKey(0);
}

void Map::findMiddleOfRooms(int threshold)
{
    vector<wall> walls;
    wall temp;
    int whitepixel = 0;
    bool newWall = true;
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            if(newWall)
            {
                temp.start.x = j;
                temp.start.y = i;
                newWall = false;
            }
            if(map.at<uchar>(i,j) == '0')
            {
                whitepixel++;
            }
            if(whitepixel > threshold)
            {
                whitepixel=0;
                temp.end.x = j;
                temp.end.y = i;
                newWall = true;
                walls.push_back(temp);
            }
            else
            {

            }
        }
    }
    for(int i = 0; i < walls.size(); i++)
    {
        cout << "Start x : " << walls[i].start.x << "Start y : " << walls[i].start.y <<  "End x : " << walls[i].end.x << "End y : " << walls[i].end.y << endl;
    }
}

Map::~Map()
{

}


