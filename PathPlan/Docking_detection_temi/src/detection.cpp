/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include <time.h>
#include "ros/ros.h"   
#include <geometry_msgs/Point.h>
#include <vector>
#include <map>
#include "LaserType.cpp"


//--Objects--//
LaserType laser_obj_;
//--VARIABLES--//
ros::Time procedure_time;
//docking geometry
#define DOCKING_WIDTH                           0.15
#define DOCKING_WIDTH_TOLERANCE_INTENSITY       0.08           
#define DOCK_DISTANCE_FROM_WALL                 0.06
//param values
double start_scan_angle_from_robot;
double end_scan_angle_from_robot;
//line vectors
std::vector <LineType> line_vectors_;
std::vector <LineType> docking_lines_full_detection_vec;
std::vector <LineType> docking_lines_by_length_vec;
std::vector <LineType> docking_lines_by_neighbors_vec;


//------Running Funcation-------//
void InitIterationParams()
{
    line_vectors_.clear();
    docking_lines_by_length_vec.clear();
    docking_lines_by_neighbors_vec.clear();
    docking_lines_full_detection_vec.clear();
}

void FormLinesInReverseBynaricMethod()
{
    if (laser_obj_.filtered_laser_scans_by_desired_angles_map.empty())
    {
        return;
    }
    line_vectors_.clear();
    LineType testing_line;
    std::vector <PointToLineType>  all_laser_points;
    for (const auto& n : laser_obj_.filtered_laser_scans_by_desired_angles_map)
    {
        double angle_from_robot = n.first;
        geometry_msgs::Point laser_point = n.second.scan_point;
        PointToLineType point_to_line;
        point_to_line.points_coordinates.point.x = laser_point.x;
        point_to_line.points_coordinates.point.y = laser_point.y;
        all_laser_points.push_back(point_to_line);
    }
    int min_index, mid_index, max_index, start_next_line_index, end_index;
    int num_of_points = all_laser_points.size();
    min_index = 0;
    end_index = num_of_points - 1;
    start_next_line_index = min_index;
    int temp_counter = 0;
    while (end_index - start_next_line_index >= 1)
    {
        std::vector <PointToLineType> testing_points_vec;
        auto min_it = all_laser_points.begin();
        auto second_point_it = all_laser_points.begin();
        std::advance(min_it, start_next_line_index);
        std::advance(second_point_it, start_next_line_index+1);
        bool found_line = false;
        min_index = start_next_line_index;
        max_index = std::min(min_index + 1, end_index);
        while (min_index <= max_index)
        {
            temp_counter ++;
            mid_index = (max_index + min_index) % 2 ? (max_index + min_index) / 2 + 1 : (max_index + min_index) / 2; 
            min_it = all_laser_points.begin();
            auto mid_it = all_laser_points.begin();
            std::advance(min_it, min_index);
            std::advance(mid_it, mid_index);
            testing_points_vec.clear();
            testing_points_vec.insert(testing_points_vec.end(), min_it, mid_it+1);
            bool points_are_on_line = testing_line.PointsAreOnTheLine(testing_points_vec);
            int advance_fator = (max_index - min_index)*2 + 1;
            if (points_are_on_line)
            {
                testing_line.AddPointsVecToLine(testing_points_vec);
                min_index = mid_index + 1;
                start_next_line_index = min_index;
                max_index = std::min(min_index + advance_fator, end_index);
                found_line = true;
            }
            else
            {
                int stepback_fator = std::max(((double)(max_index - min_index) / 2.0), 1.0); 
                max_index = std::min(max_index - stepback_fator, end_index);
            }
        }
        if (found_line)
        {
            line_vectors_.push_back(testing_line);
            testing_line.Clear();
            max_index = std::min(min_index + 1, end_index);
        }
        else
        {
            start_next_line_index = min_index + 1;
        }
    }
}

void setLinesIDs()
{
    int id = 1;
    for (int i=0; i<line_vectors_.size(); i++)
    {
        LineType& current_line = line_vectors_.at(i);
        current_line.line_id = id;
        id++;
    }
}

std::vector <LineType> detectDockLinesByLength(std::vector <LineType> line_vectors)
{
    std::vector <LineType> docking_by_length_vector;
    docking_by_length_vector.clear();
    double docking_geometric_width = DOCKING_WIDTH;
    std::map <double, LineType> distance_from_robot_line_map;
    for(int i=0; i<line_vectors.size(); i++)
    {
        LineType& current_line = line_vectors.at(i);
        bool docking_detection_by_length = current_line.line_length >= docking_geometric_width-0.05 && current_line.line_length <= docking_geometric_width + 0.04;
        bool docking_detection_by_distance_from_robot = current_line.distance_from_robot < 1.0; 
        if (docking_detection_by_length && docking_detection_by_distance_from_robot)  {
            distance_from_robot_line_map.insert( std::pair<double, LineType>(current_line.distance_from_robot, current_line) );
        }
    }
    for (const auto& n : distance_from_robot_line_map) {
        LineType line_to_push = n.second;
        docking_by_length_vector.push_back(line_to_push);
    }
    return docking_by_length_vector;
}

std::vector <LineType> detectDockLinesByNeighbors(std::vector <LineType> line_vectors)
{
    std::vector <LineType> docking_by_neighbors_vector;
    docking_by_neighbors_vector.clear();
    std::map <double, LineType> distance_from_robot_line_map;
    if (line_vectors.size() >= 2) {
        for (int i=0; i<line_vectors.size()-1; i++)
        {
            LineType prev_line = i-1 > -1 ? line_vectors.at(i-1) : line_vectors.at(i) ;
            LineType current_line = line_vectors.at(i);
            LineType next_line = line_vectors.at(i+1);
            double prev_current_lines_yaw_diff = wrapAngleToPi(current_line.line_yaw - prev_line.line_yaw);
            double current_next_lines_yaw_diff = wrapAngleToPi(next_line.line_yaw - current_line.line_yaw);
            const double thresh_max_yaw_error_diff = 0.0875; //[rad] ~5 degrees
            bool docking_detection_by_yaw_diff = abs(current_next_lines_yaw_diff) <  thresh_max_yaw_error_diff && abs(prev_current_lines_yaw_diff) <  thresh_max_yaw_error_diff;
            double prev_current_lines_vertical_distance_diff = abs(current_line.line_first_point.x - prev_line.line_last_point.x); //distance diff between first line last point to last line last point
            double current_next_lines_vertical_distance_diff = abs(next_line.line_first_point.x - current_line.line_last_point.x); //distance diff between first line last point to last line last point
            double prev_current_distance_diff_detect_error = prev_current_lines_vertical_distance_diff - DOCK_DISTANCE_FROM_WALL;
            double distance_diff_detect_error = current_next_lines_vertical_distance_diff - DOCK_DISTANCE_FROM_WALL;
            const double thersh_max_distance_diff_detect_error_upper = 0.04; //[meter] tollerance from distance from wall
            const double thersh_max_distance_diff_detect_error_lower = 0.03; //[meter] tollerance from distance from wall
            bool prev_current_docking_detection_by_distance_diff = prev_current_distance_diff_detect_error > 0 ? abs(prev_current_distance_diff_detect_error) <= thersh_max_distance_diff_detect_error_upper :  abs(distance_diff_detect_error) <= thersh_max_distance_diff_detect_error_lower; //[meter] tollerance from distance from wall
            bool docking_detection_by_distance_diff = distance_diff_detect_error > 0 ? abs(distance_diff_detect_error) <= thersh_max_distance_diff_detect_error_upper :  abs(distance_diff_detect_error) <=thersh_max_distance_diff_detect_error_lower;
            bool docking_by_neighbors_detection_by_distance = prev_current_docking_detection_by_distance_diff && docking_detection_by_distance_diff;
            bool docking_detection_by_distance_from_robot = current_line.distance_from_robot < 1.0; //[meter]only id distance from robot is smaller than
            if (docking_detection_by_yaw_diff && docking_by_neighbors_detection_by_distance && docking_detection_by_distance_from_robot)
            {
                distance_from_robot_line_map.insert( std::pair<double, LineType>(current_line.distance_from_robot, current_line) );
            }
        }
    }
    for (const auto& n : distance_from_robot_line_map) {
        LineType line_to_push = n.second;
        docking_by_neighbors_vector.push_back(line_to_push);
    }
    return docking_by_neighbors_vector;
}

std::vector <LineType> mergeDetectionVectors()
{
    std::vector <LineType> merged_detection_vector;
    merged_detection_vector.clear();
    int i = 0;
    while (i < docking_lines_by_length_vec.size())
    {
        LineType& current_length_line = docking_lines_by_length_vec.at(i);
        int j = 0;
        bool found_a_match = false;
        while (j < docking_lines_by_neighbors_vec.size() && !found_a_match)
        {
            LineType& current_neighbors_line = docking_lines_by_neighbors_vec.at(j);
            if (current_length_line.line_id == current_neighbors_line.line_id)
            {
                if (merged_detection_vector.empty())
                {
                    merged_detection_vector.push_back(current_length_line);
                }
                else 
                {
                    int merged_vec_size = merged_detection_vector.size();
                    LineType& last_line = merged_detection_vector.at(merged_vec_size -1);
                    if (current_length_line.distance_from_robot < last_line.distance_from_robot) 
                    {
                        merged_detection_vector.pop_back();
                        merged_detection_vector.push_back(current_length_line);
                        merged_detection_vector.push_back(last_line);
                    }
                    else
                    {
                        merged_detection_vector.push_back(current_length_line);
                    }
                }
                found_a_match = true;
            }
            j++;
        }
        i++;
    }
    return merged_detection_vector;
}

void DetectDocking()
{
    double start_angle = start_scan_angle_from_robot;
    double end_angle = end_scan_angle_from_robot;
    std::cout<<"debug2"<<std::endl;
    laser_obj_.UpdateObjectMembers(start_angle, end_angle);
    std::cout<<"debug3"<<std::endl;
    FormLinesInReverseBynaricMethod();
    setLinesIDs(); //sets ID to Each line
    docking_lines_by_length_vec = detectDockLinesByLength(line_vectors_); //detects docking line by line length
    docking_lines_by_neighbors_vec = detectDockLinesByNeighbors(line_vectors_); //detects docking line by line neighbors
    docking_lines_full_detection_vec = mergeDetectionVectors();
    std::cout<<"debug4 value is: "<<docking_lines_by_length_vec.size()<<std::endl;
    std::cout<<"debug5 value is: "<<docking_lines_by_neighbors_vec.size()<<std::endl;
    std::cout<<"debug6 value is: "<<docking_lines_full_detection_vec.size()<<std::endl;
}

int main()
{
    std::cout<<"Welcome to docking debug system"<<std::endl;
    InitIterationParams();
    std::cout<<"debug1"<<std::endl;
    DetectDocking();
    std::cout<<"debug4"<<std::endl;
    return 0;
}
