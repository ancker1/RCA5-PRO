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
        // Not obstacle up and left and up and right
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
    double offsetx = 7.0;
    double offsety = 5.0;
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
    struct sortx {
        bool operator() (Point pt1, Point pt2) { return (pt1.x < pt2.x);}
    } myobjectx;

    struct sorty {
        bool operator() (Point pt1, Point pt2) { return (pt1.y < pt2.y);}
    } myobjecty;


    vector<Point_<double>> convertedGoals;
    vector<Point> tempGoals;
    for(size_t i = 0; i < upperGoals.size(); i++)
    {
        tempGoals.push_back(upperGoals[i]);
    }
    for(size_t i = 0; i < lowerGoals.size(); i++)
    {
        tempGoals.push_back(lowerGoals[i]);
    }
    double widthScale = 20.0/14.0;
    double heightScale = 15.0/11.0;
    double offsetx = 6.5;
    double offsety = 4.5;
    Point_<double> convertedPoint;
    vector<Point> deletes;

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
        convertedGoals.push_back(tempGoals[i]);
    }
    return convertedGoals;
}

Map::~Map()
{

}

/****************************************************
 *  BUSHFIRE
 * *************************************************/

Mat Map::bushfire_img(Mat &img) {
    Mat binary_img = img.clone();
    binarize_img(binary_img);
    make_bushfire_grid(binary_img);
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

void Map::make_bushfire_grid(Mat &img) {
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
