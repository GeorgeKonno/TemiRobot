/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include "ros/ros.h"   
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "Best_fit_temi/BestFit.cpp"
#include "Best_fit_temi/Double.cpp"
#include "Best_fit_temi/Shapes.cpp"
#include "PointToLineType.cpp"
#include <vector>


class LineType
{
public:
    LineType();
    LineType(PointToLineType first_point, PointToLineType second_point);
    void InitLineVariables();
    void Clear();
    void CalculateBestFit(std::vector <PointToLineType> &input_point_vector, std::vector <PointToLineType> &best_fit_point_vector, double &M_slope, double &B_y_intercept);
    void CalculateBestFitMinimized(std::vector <PointToLineType> &input_point_vector, double &M_slope, double &B_y_intercept);
    int getNumberOfPointsInLine();
    double getAllowedSdRelativeErrorPrecentage();
    void setLineBasicDefinition(double calculated_slope, double calculated_y_intercept);
    void setAllowedSdRelativeErrorPrecentage(double desired_value);
    bool PointIsInTheLine(PointToLineType test_point); //returns whether point is in the points vector that consists the line
    bool PointIsOnTheLine(PointToLineType test_point); //returns whether new point is on the line
    bool PointsAreOnTheLine(std::vector <PointToLineType> desired_points);
    double CalculatePointPerpendicularDistanceFromLine(PointToLineType point_to_test, double M_slope, double B_y_intercept);
    double CalculatePointPerpendicularDistanceFromLine(geometry_msgs::Point point_to_test);
    void CalculateLineStdevAndPointsVariance(); //calculates line standart devation;
    void CalculateLineStdevAndPointsVariance(std::vector <PointToLineType> points_vector, double points_average_distance_from_line, double& ref_Stdev, double& ref_variance); //calculates line standart devation;
    void CalculateLineAdditionalInfo();
    geometry_msgs::Point getPointFromLineRatio(double desired_ratio);
    geometry_msgs::Pose getPoseOrthogonolFromLineRatio(double desired_ratio);
    double getLineLength();
    void AddPointToLine (PointToLineType point_to_add); //adds point to line
    void AddPointsVecToLine (std::vector <PointToLineType> point_vec);
    void RemovePointFromLine (PointToLineType point_to_remove);
    void UpdateLineSpecs(); //updates line details and variables
    void PrintLineRawPoints();
    void PrintLineFitPoints();
    void PrintPointsDistanceFromLine();
    double line_slope; //slope of the line Y=M*X+B - "M"
    double line_y_intercept; //y_intercept of the line Y=M*X+B - "B"
    std::vector <PointToLineType> input_points_vec; //points that build the best fit line
    std::vector <PointToLineType> bestfit_points_vec; //points that build the best fit line
    double best_fit_current_stdev;
    double stdev_distance_from_line; //standard deviation of the points distance from the line
    double average_points_distance; // average points distance from the line;
    double line_length; //line length from the first point to the last
    double line_yaw;
    geometry_msgs::Point line_first_point;
    geometry_msgs::Point line_last_point;
    geometry_msgs::Point line_middle_point;
    double distance_from_robot;
    double distance_to_neighbor_line_clockwise; //distance to the nearest line clockwise
    double distance_to_neighbor_line_counter_clockwise; //distance to the nearest line ccounter
    int line_id; //line id
    int minimun_required_points_in_line;
    std::string line_frame_id;
private:
    double max_allowed_sd_relative_error_precentage;
    double max_allowed_sd_value_;
    double x_coord_total; //sum of the x_cordintates of the points vector;
    double y_coord_total; //sum of the y_cordintates of the points vector;
    double x_coord_average; //avergae of the X codinates of the points vector
    double y_coord_average; //avergae of the Y codinates of the points vector
    double total_distance_point_from_line; //sum of the distance of the points vector from line;
};