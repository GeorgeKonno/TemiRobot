/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include "PointToLineType.h"


PointToLineType::PointToLineType()
{
    angle_from_robot = 0; //angle from robot point
    distance_from_robot = 0; //euclidean distance from robot
    distance_from_associated_line = 0; //distance from associated
    SD_line_without_point = 0; //standard deviation of the points distance from the line without this point
    SD_line_with_point = 0; //standard deviation of the points distance from the line with this point
    point_distance_from_line_variance = 0; //Variance of the point distance trom line
    best_fit_residuals = 0;
}

void PointToLineType::PrintPointSpecs()
{

}