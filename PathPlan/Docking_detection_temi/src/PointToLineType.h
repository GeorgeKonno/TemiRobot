/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include "ros/ros.h"    
#include <geometry_msgs/PointStamped.h>


class PointToLineType
{
public:
    PointToLineType();
    void PrintPointSpecs();
    geometry_msgs::PointStamped points_coordinates; //point X,Y,Z Coordinates and STAMPED FRAME
    double angle_from_robot; //angle from robot point
    double distance_from_robot; //euclidean distance from robot
    double distance_from_associated_line; //distance from associated
    double SD_line_without_point; //standard deviation of the points distance from the line without this point
    double SD_line_with_point; //standard deviation of the points distance from the line with this point
    double point_distance_from_line_variance; //Variance of the point distance trom line
    double best_fit_residuals;

private:
};