/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include <math.h>
#include "LineType.h"
#include <tf/transform_datatypes.h>


#define LASER_FRAME ("laser")


double wrapAngleToPi(double angle)
{
    angle = fmod(angle + M_PI, 2.0*M_PI);
    if (angle < 0.0)
        angle += 2.0*M_PI;
    return angle - M_PI;
}

double getEuclideanDistance(double p1[], double p2[])
{
    return (p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]);
}

double getEuclideanDistance_ros(const geometry_msgs::Point &pnt2, const geometry_msgs::Point &pnt1)
{
    double p1[] = {pnt1.x, pnt1.y};
    double p2[] = {pnt2.x, pnt2.y};
    return getEuclideanDistance(p1, p2);
}

LineType::LineType()
{
    InitLineVariables();
}

LineType::LineType(PointToLineType first_point, PointToLineType second_point)
{
    InitLineVariables();
    AddPointToLine(first_point);
    AddPointToLine(second_point);
    UpdateLineSpecs();
}

void LineType::InitLineVariables()
{
    input_points_vec.clear();
    bestfit_points_vec.clear();
    minimun_required_points_in_line = 3;
    max_allowed_sd_relative_error_precentage = 2.0;//5.0;//15.0;
    max_allowed_sd_value_ = 0.004;//0.005
    stdev_distance_from_line = 0;
    average_points_distance = 0;
    best_fit_current_stdev = 0;
    line_frame_id = LASER_FRAME;
    x_coord_total = 0; //sum of the x_cordintates of the points vector;
    y_coord_total = 0; //sum of the y_cordintates of the points vector;
    x_coord_average = 0; //avergae of the X codinates of the points vector
    y_coord_average = 0; //avergae of the Y codinates of the points vector
}

void LineType::Clear()
{
    InitLineVariables();
}

void LineType::CalculateBestFit(std::vector<PointToLineType> &input_point_vector, std::vector<PointToLineType> &best_fit_point_vector, double &M_slope, double &B_y_intercept)
{
    int num_of_points = input_point_vector.size();
    double points[num_of_points*2];
    int points_index = 0;
    for (int i = 0; i < num_of_points; i++)
    {
        geometry_msgs::Point converted_point_for_best_fit;
        converted_point_for_best_fit.y = input_point_vector.at(i).points_coordinates.point.x;
        converted_point_for_best_fit.x = -1 * input_point_vector.at(i).points_coordinates.point.y;
        points[points_index] = converted_point_for_best_fit.x;
        points[points_index+1] = converted_point_for_best_fit.y;
        points_index = points_index + 2;
    }
    BestFitIO input;
    input.numPoints = num_of_points;
    input.points = points;
    BestFitIO output;
    output.points = new double[num_of_points*2]; // allocate space for adjusted points.
    output.wantAdjustedObs = true;  // output.points will get populated
    output.wantResiduals = true;    // output.residuals will get allocated and populated
    int type = BestFitFactory::Line;
    std::ostream bitBucket(0);
    bitBucket;
    BestFit *b = BestFitFactory::Create(type, bitBucket);//std::cout);
    try
    {
        b->Compute(input, output);
    }
    catch (...)
    {
    }
    best_fit_point_vector.clear();
    PointToLineType point_to_push;
    points_index = 0;
    for (int i = 0; i < num_of_points; i++) //output for
    {
        geometry_msgs::Point converted_point_from_best_fit;
        converted_point_from_best_fit.x = output.points[points_index+1];
        converted_point_from_best_fit.y = -1 * output.points[points_index];;
        point_to_push.points_coordinates.point.x = converted_point_from_best_fit.x;
        point_to_push.points_coordinates.point.y = converted_point_from_best_fit.y;

        best_fit_point_vector.push_back(point_to_push);
        points_index = points_index + 2;
    }

    M_slope = (best_fit_point_vector.at(num_of_points-2).points_coordinates.point.y - best_fit_point_vector.at(num_of_points-1).points_coordinates.point.y)/((best_fit_point_vector.at(num_of_points-2).points_coordinates.point.x - best_fit_point_vector.at(num_of_points-1).points_coordinates.point.x));
    B_y_intercept = point_to_push.points_coordinates.point.y - M_slope * point_to_push.points_coordinates.point.x;
    delete b;
    delete [] output.residuals;
    delete [] output.points;
}

void LineType::CalculateBestFitMinimized(std::vector <PointToLineType> &input_point_vector, double &M_slope, double &B_y_intercept)
{
    int num_of_points = input_point_vector.size();
    double points[num_of_points*2];
    int points_index = 0;
    for (int i = 0; i < num_of_points; i++)
    {
        geometry_msgs::Point converted_point_for_best_fit;
        converted_point_for_best_fit.y = input_point_vector.at(i).points_coordinates.point.x;
        converted_point_for_best_fit.x = -1 * input_point_vector.at(i).points_coordinates.point.y;
        points[points_index] = converted_point_for_best_fit.x;
        points[points_index+1] = converted_point_for_best_fit.y;
        points_index = points_index + 2;
    }
    BestFitIO input;
    input.numPoints = num_of_points;
    input.points = points;
    BestFitIO output;
    output.points = new double[num_of_points*2]; // allocate space for adjusted points.
    output.wantAdjustedObs = false;  // output.points will get populated
    output.wantResiduals = false;    // output.residuals will get allocated and populated
    int type = BestFitFactory::Line;
    std::ostream bitBucket(0);
    bitBucket;
    BestFit *b = BestFitFactory::Create(type, bitBucket);//std::cout);
    try
    {
        b->Compute(input, output);
    }
    catch (...)
    {
    }
    M_slope = output.outputFields[BestFitIO::LineGradient]; //setting the line basic properties
    B_y_intercept = output.outputFields[BestFitIO::LineYIntercept]; //setting the line basic properties
    delete b;
    delete [] output.residuals;
    delete [] output.points;

}

bool LineType::PointIsOnTheLine(PointToLineType test_point)
{
    bool point_is_on_the_line = true;
    std::vector <PointToLineType> temp_vector_to_test = input_points_vec;
    temp_vector_to_test.push_back(test_point);
    if (temp_vector_to_test.size() <= 2)
    {
        point_is_on_the_line = true;
    }
    else
    {
        double temp_M_slope, temp_B_y_intercept;
        CalculateBestFitMinimized(temp_vector_to_test, temp_M_slope, temp_B_y_intercept);
        int num_of_points = std::max((int)temp_vector_to_test.size(), 1);
        double temp_total_distance_point_from_line = 0;
        for (int i = 0; i < num_of_points; i++)
        {
            PointToLineType& current_point = temp_vector_to_test.at(i);
            current_point.distance_from_associated_line = CalculatePointPerpendicularDistanceFromLine(current_point, temp_M_slope, temp_B_y_intercept);
            temp_total_distance_point_from_line = temp_total_distance_point_from_line +  current_point.distance_from_associated_line;
        }
        double temp_average_points_distance = temp_total_distance_point_from_line/num_of_points;
        double temp_stev, temp_variance;
        CalculateLineStdevAndPointsVariance(temp_vector_to_test, temp_average_points_distance, temp_stev, temp_variance);
        double std_true_value = stdev_distance_from_line == 0.0 ? 0.0001 : stdev_distance_from_line;
        if (num_of_points == 3)
        {
            std_true_value = temp_stev;
        }
        double relative_error_precent = ((temp_stev - std_true_value)/std_true_value)*100;
        bool point_is_on_line_condition = relative_error_precent <= max_allowed_sd_relative_error_precentage || temp_stev <= max_allowed_sd_value_;
        if (point_is_on_line_condition)
        {
            point_is_on_the_line = true;
        }
        else
        {
            point_is_on_the_line = false;
        }
    }
    return point_is_on_the_line;
}

bool LineType::PointsAreOnTheLine(std::vector <PointToLineType> desired_points)
{
    bool points_are_on_the_line = true;
    std::vector <PointToLineType> temp_best_fit_vector;
    std::vector <PointToLineType> temp_vector_to_test = input_points_vec;
    temp_vector_to_test.insert(temp_vector_to_test.end(), desired_points.begin(), desired_points.end());
    if (temp_vector_to_test.size() < 2)
    {
        points_are_on_the_line = true;
    }
    else if (temp_vector_to_test.size() == 2)
    {
        points_are_on_the_line = true;
    }
    else
    {
        double temp_M_slope, temp_B_y_intercept;
        CalculateBestFit(temp_vector_to_test, temp_best_fit_vector, temp_M_slope, temp_B_y_intercept);
        int num_of_points = std::max((int)temp_vector_to_test.size(), 1);
        double temp_total_distance_point_from_line = 0;
        for (int i = 0; i < num_of_points; i++)
        {
            PointToLineType& current_point = temp_vector_to_test.at(i);
            current_point.distance_from_associated_line = CalculatePointPerpendicularDistanceFromLine(current_point, temp_M_slope, temp_B_y_intercept);
            temp_total_distance_point_from_line = temp_total_distance_point_from_line +  current_point.distance_from_associated_line;
        }
        double temp_average_points_distance = temp_total_distance_point_from_line/num_of_points;
        double temp_stev, temp_variance;
        CalculateLineStdevAndPointsVariance(temp_vector_to_test, temp_average_points_distance, temp_stev, temp_variance);
        double std_true_value = stdev_distance_from_line == 0.0 ? 0.0000000001 : stdev_distance_from_line;
        double relative_error_precent = ((temp_stev - std_true_value)/std_true_value)*100;
        if (stdev_distance_from_line == 0)
        {
            double temp_M_slope_3_point, temp_B_y_intercept_3_point;
            std::vector <PointToLineType> three_point_vec;
            std::vector <PointToLineType> best_fit_three_point_vec;
            three_point_vec.insert(three_point_vec.end(), temp_vector_to_test.begin(), temp_vector_to_test.begin()+3);
            CalculateBestFit(three_point_vec, best_fit_three_point_vec, temp_M_slope, temp_B_y_intercept);
            num_of_points = std::max((int)three_point_vec.size(), 1);
            double temp_total_distance_point_from_line_3_point = 0;
            for (int i = 0; i < num_of_points; i++)
            {
                PointToLineType& current_point = three_point_vec.at(i);
                current_point.distance_from_associated_line = CalculatePointPerpendicularDistanceFromLine(current_point, temp_M_slope_3_point, temp_B_y_intercept_3_point);
                temp_total_distance_point_from_line_3_point = temp_total_distance_point_from_line_3_point +  current_point.distance_from_associated_line;
            }
            double temp_average_points_distance_3_point = temp_total_distance_point_from_line_3_point/num_of_points;
            double temp_stev_3_point, temp_variance_3_point;
            CalculateLineStdevAndPointsVariance(three_point_vec, temp_average_points_distance_3_point, temp_stev_3_point, temp_variance_3_point);
            if (temp_stev_3_point <= max_allowed_sd_value_)
            {
                relative_error_precent = ((temp_stev - temp_stev_3_point)/temp_stev_3_point)*100;
            }
        }
        points_are_on_the_line = relative_error_precent <= max_allowed_sd_relative_error_precentage || temp_stev <= max_allowed_sd_value_;
    }
    return points_are_on_the_line;
}

int LineType::getNumberOfPointsInLine()
{
    return input_points_vec.size();
}

double LineType::getAllowedSdRelativeErrorPrecentage()
{
    return max_allowed_sd_relative_error_precentage;
}

void LineType::setLineBasicDefinition(double calculated_slope, double calculated_y_intercept)
{
    line_slope = calculated_slope;
    line_y_intercept = calculated_y_intercept;
}

void LineType::setAllowedSdRelativeErrorPrecentage(double desired_value)
{
    max_allowed_sd_relative_error_precentage = desired_value;
}

void LineType::AddPointToLine(PointToLineType point_to_add)
{
    input_points_vec.push_back(point_to_add);
    UpdateLineSpecs();
}

void LineType::AddPointsVecToLine(std::vector <PointToLineType> point_vec)
{
    input_points_vec.insert(input_points_vec.end(), point_vec.begin(), point_vec.end());
    UpdateLineSpecs();
}

void LineType::RemovePointFromLine (PointToLineType point_to_remove)
{
    int num_of_points = input_points_vec.size();
    bool found_point = false;
    int i;
    for (i = 0; i < num_of_points; i++)
    {
        bool x_coord_is_eqaul = input_points_vec.at(i).points_coordinates.point.x == point_to_remove.points_coordinates.point.x;
        bool y_coord_is_eqaul = input_points_vec.at(i).points_coordinates.point.y == point_to_remove.points_coordinates.point.y;
        found_point = x_coord_is_eqaul && y_coord_is_eqaul;
        if (found_point)
        {
            break;
        }
    }
    if (found_point)
    {
        input_points_vec.erase(input_points_vec.begin()+i-1);
        UpdateLineSpecs();
        return;
    }
    else
    {
    }
}


double LineType::CalculatePointPerpendicularDistanceFromLine(PointToLineType point_to_test, double M_slope, double B_y_intercept)
{
    double x_coordinate = point_to_test.points_coordinates.point.x;
    double y_coordinate = point_to_test.points_coordinates.point.y;
    double m = M_slope;//line_slope;
    double b = B_y_intercept;//line_y_intercept;
    double normal_factor = (sqrt(m*m +1));
    normal_factor = normal_factor == 0 ? 1 : normal_factor;
    double distance = abs((m*x_coordinate - y_coordinate + b )/normal_factor);
    return distance;
}

double LineType::CalculatePointPerpendicularDistanceFromLine(geometry_msgs::Point point_to_test)
{
    double x_coordinate = point_to_test.x;
    double y_coordinate = point_to_test.y;
    double m = line_slope;//line_slope;
    double b = line_y_intercept;//line_y_intercept;
    double normal_factor = (sqrt(m*m +1));
    normal_factor = normal_factor == 0 ? 1 : normal_factor;
    double distance = abs((m*x_coordinate - y_coordinate + b )/normal_factor);
    return distance;
}

void LineType::CalculateLineStdevAndPointsVariance()
{
    int num_of_points = input_points_vec.size();
    double sum_of_variance = 0;
    for (int i = 0; i < num_of_points; i++)
    {
        PointToLineType& current_point = input_points_vec.at(i);
        double point_distance_from_line =  current_point.distance_from_associated_line;
        current_point.point_distance_from_line_variance = (average_points_distance - point_distance_from_line)*(average_points_distance - point_distance_from_line);
        sum_of_variance = sum_of_variance + current_point.point_distance_from_line_variance;
    }
    stdev_distance_from_line = sqrt( (1/(double)num_of_points) * sum_of_variance);
}

void LineType::CalculateLineStdevAndPointsVariance(std::vector<PointToLineType> points_vector, double points_average_distance_from_line, double& ref_Stdev, double& ref_variance)
{
    int num_of_points = points_vector.size();
    ref_variance = 0;
    for (int i = 0; i < num_of_points; i++)
    {
        PointToLineType& current_point = points_vector.at(i);
        double point_distance_from_line =  current_point.distance_from_associated_line;
        current_point.point_distance_from_line_variance = (points_average_distance_from_line - point_distance_from_line)*(points_average_distance_from_line - point_distance_from_line);
        ref_variance = ref_variance + current_point.point_distance_from_line_variance;
    }
    ref_Stdev = sqrt( (1/(double)num_of_points) * ref_variance);
}

geometry_msgs::Point LineType::getPointFromLineRatio(double desired_ratio)
{
    geometry_msgs::Point desired_point;
    if (bestfit_points_vec.empty())
    {
        return desired_point;
    }
    int number_of_points = getNumberOfPointsInLine();
    geometry_msgs::Point line_first_point = bestfit_points_vec.at(0).points_coordinates.point;
    geometry_msgs::Point line_last_point = bestfit_points_vec.at(number_of_points-1).points_coordinates.point;
    tf::Vector3 line_vector = tf::Vector3(line_last_point.x - line_first_point.x, line_last_point.y - line_first_point.y, 0);
    double line_yaw = wrapAngleToPi(atan2(line_vector.getY(), line_vector.getX()));
    double line_length = getEuclideanDistance_ros(line_first_point, line_last_point);
    desired_point.x = line_first_point.x + line_vector.normalized().getX()*line_length * desired_ratio;
    desired_point.y = line_first_point.y + line_vector.normalized().getY()*line_length * desired_ratio;
    desired_point.z = 0;
    return desired_point;
}

geometry_msgs::Pose LineType::getPoseOrthogonolFromLineRatio(double desired_ratio)
{
    geometry_msgs::Pose desired_Pose;
    desired_Pose.position = getPointFromLineRatio(desired_ratio);
    double line_orthogonol_yaw = wrapAngleToPi(line_yaw + M_PI_2);
    desired_Pose.orientation = tf::createQuaternionMsgFromYaw(line_orthogonol_yaw); //finel dest orinetation
    return desired_Pose;
}

double LineType::getLineLength()
{
    int number_of_points = getNumberOfPointsInLine();
    geometry_msgs::Point line_first_point = bestfit_points_vec.at(0).points_coordinates.point;
    geometry_msgs::Point line_last_point = bestfit_points_vec.at(number_of_points-1).points_coordinates.point;
    double line_length = getEuclideanDistance_ros(line_first_point, line_last_point);
    return line_length;
}

void LineType::UpdateLineSpecs()
{
    int num_of_points = std::max((int)input_points_vec.size(), 1);
    if (num_of_points == 1)
    {
        line_slope = 0;
        line_y_intercept = 0;
        bestfit_points_vec = input_points_vec;
    }
    else if (num_of_points == 2) //line has only 2 points or less
    {
        geometry_msgs::Point first_point = input_points_vec.at(0).points_coordinates.point;
        geometry_msgs::Point second_point = input_points_vec.at(1).points_coordinates.point;
        line_slope = (second_point.y - first_point.y)/(second_point.x - first_point.x);
        line_y_intercept = second_point.y - second_point.x*line_slope;
        bestfit_points_vec = input_points_vec;
    }
    else
    {
        CalculateBestFit(input_points_vec, bestfit_points_vec, line_slope, line_y_intercept); //calculate line and params
    }
    x_coord_total = 0;
    y_coord_total = 0;
    total_distance_point_from_line = 0;
    for (int i = 0; i < num_of_points; i++)
    {
        if (num_of_points <= 2) //line has only 2 points or less
        {
            input_points_vec.at(i).distance_from_associated_line = 0;
            total_distance_point_from_line = 0;
        }
        else
        {
            input_points_vec.at(i).distance_from_associated_line = CalculatePointPerpendicularDistanceFromLine(input_points_vec.at(i), line_slope, line_y_intercept);
            total_distance_point_from_line = total_distance_point_from_line +  input_points_vec.at(i).distance_from_associated_line;
        }
        x_coord_total = x_coord_total + input_points_vec.at(i).points_coordinates.point.x;
        y_coord_total = y_coord_total + input_points_vec.at(i).points_coordinates.point.y;
    }
    x_coord_average = x_coord_total/num_of_points;
    y_coord_average = y_coord_total/num_of_points;
    average_points_distance = total_distance_point_from_line/num_of_points;
    CalculateLineStdevAndPointsVariance(); //calculates points distance from line standard devation and points variaance.
    CalculateLineAdditionalInfo();
}

void LineType::CalculateLineAdditionalInfo()
{

    line_length = getLineLength();
    line_first_point = bestfit_points_vec.at(0).points_coordinates.point;
    line_last_point = getPointFromLineRatio(1.0); //means last point // bestfit_points_vec.at(bestfit_points_vec.size()-1).points_coordinates.point
    line_middle_point = getPointFromLineRatio(0.5); //0.5 means middle point
    geometry_msgs::Point base_link_point;
    base_link_point.x = 0;
    base_link_point.y = 0;
    distance_from_robot = getEuclideanDistance_ros(base_link_point, line_middle_point);//CalculatePointPerpendicularDistanceFromLine(base_link_point);
    tf::Vector3 line_vector = tf::Vector3(line_last_point.x - line_first_point.x, line_last_point.y - line_first_point.y, 0);
    line_yaw = wrapAngleToPi(atan2(line_vector.getY(), line_vector.getX()));

}


void LineType::PrintLineRawPoints()
{
    int num_of_points = input_points_vec.size();
    for (int i = 0; i < num_of_points; i++)
    {
        input_points_vec.at(i).PrintPointSpecs();
    }
}

void LineType::PrintLineFitPoints()
{
    int num_of_points = bestfit_points_vec.size();
    for (int i = 0; i < num_of_points; i++)
    {
        bestfit_points_vec.at(i).PrintPointSpecs();
    }
}

void LineType::PrintPointsDistanceFromLine()
{
    int num_of_points = input_points_vec.size();
    for (int i = 0; i < num_of_points; i++)
    {
        double point_distance_from_line = input_points_vec.at(i).distance_from_associated_line;
    }
}


