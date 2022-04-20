/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include "LaserType.h"
#include "ros/ros.h"
#include "LineType.cpp"


#define D_2_PI         6.28318530717958647692  /* 2*pi */


LaserType::LaserType()
{
    initParams();
}

LaserType::~LaserType()
{
}

double wrapAngleTo2Pi(double angle)
{
    angle = fmod(angle, 2.0*M_PI);
    if (angle < 0.0)
        angle += 2.0*M_PI;
    return angle;
}

void LaserType::initParams()
{
    original_laser_scan_vec.clear();
    raw_laser_scans_by_desired_angles_map.clear();
    filtered_laser_scans_by_desired_angles_map.clear();
    erased_points_after_filter.clear();
}

void LaserType::updateOriginalLaserScanVector(const std::vector<float> ranges_vector, const std::vector<float> intesities_vector)
{
    original_laser_scan_vec.clear();
    int nubmer_of_points = ranges_vector.size();
    for (int i=0; i<nubmer_of_points; i++)
    {
        LaserScanType laser_current_scan;
        laser_current_scan.scan_distance = (double)ranges_vector[i];
        laser_current_scan.intensity = (int)intesities_vector[i];
        laser_current_scan.angle_from_robot = ((double)i * D_2_PI) /((double)nubmer_of_points);
        laser_current_scan.scan_point = getLaserPointFromAngleAndRange(laser_current_scan.angle_from_robot, laser_current_scan.scan_distance);
        original_laser_scan_vec.push_back(laser_current_scan);
    }
}

void LaserType::calculateRawLaserMapByAnglesFromRobot(double start_angle, double end_angle)
{
    raw_laser_scans_by_desired_angles_map.clear();
    start_angle = wrapAngleTo2Pi(start_angle);
    end_angle = wrapAngleTo2Pi(end_angle);
    for (int i=0; i<original_laser_scan_vec.size(); i++)
    {
        LaserScanType& current_scan = original_laser_scan_vec.at(i);
        if (current_scan.angle_from_robot >= start_angle && current_scan.angle_from_robot <= end_angle)
        {
            raw_laser_scans_by_desired_angles_map[current_scan.angle_from_robot] = current_scan;
        }
    }
}

void LaserType::calculateFilteredLaserMapByAnglesFromRobot()
{
    filterPointsByBasicScanProperties(); //filters nan and zero values and low and high intesities
    filterPointsByInconsistentNeighbors(); //filter by inconsistent neighbors points
    filterPointsByDistanceToNeighbors(); //filter points who are to far from each other
}

void LaserType::filterPointsByBasicScanProperties()
{
    for( const auto& n : raw_laser_scans_by_desired_angles_map )
    {
        double angle_from_robot = n.first;
        LaserScanType current_laser_scan = n.second;
        bool filter_by_intensity = current_laser_scan.intensity <= 25 || current_laser_scan.intensity >= 150;
        bool filter_condition = filter_by_intensity || std::isnan(current_laser_scan.scan_distance) || current_laser_scan.scan_point.x == 0;
        if (filter_condition)
        {
            continue;
        }
        else
        {
            filtered_laser_scans_by_desired_angles_map[angle_from_robot] = current_laser_scan;
        }
    }
}

void LaserType::filterPointsByInconsistentNeighbors()
{
    if (filtered_laser_scans_by_desired_angles_map.empty())
    {
        return;
    }
    int current_index = 0;
    int compare_index, max_index_to_compare;
    std::vector <int> index_to_filter_vec;
    auto start_it = filtered_laser_scans_by_desired_angles_map.begin();
    auto current_it = start_it;
    auto compare_it = start_it;
    while (current_index < filtered_laser_scans_by_desired_angles_map.size() -1)
    {
        current_it = start_it;
        std::advance(current_it, current_index);
        LaserScanType current_laser_scan = current_it->second;
        compare_index = std::min(current_index + 1, (int)filtered_laser_scans_by_desired_angles_map.size() - 1);
        max_index_to_compare = std::min(current_index + 4, (int)filtered_laser_scans_by_desired_angles_map.size() - 1);
        bool compare_ok = false;
        while (compare_index < max_index_to_compare && !compare_ok)
        {
            compare_it = start_it;
            std::advance(compare_it, compare_index); //try
            LaserScanType compared_laser_scan = compare_it->second;
            double distance_to_compare_point = getEuclideanDistance_ros(current_laser_scan.scan_point, compared_laser_scan.scan_point);
            const double distance_between_laser_points = 0.03; //3 [cm]//0.3 //30 [cm]
            if (distance_to_compare_point < distance_between_laser_points)
            {
                compare_ok = true;
            }
            else
            {
                index_to_filter_vec.push_back(compare_index);
                compare_index++;
            }
        }
        if (compare_ok) //reached one good - need to filter the ones between.
        {
            for( const auto& n : index_to_filter_vec)
            {
                auto erase_it = start_it;
                std::advance(erase_it, n);
                erased_points_after_filter.push_back(erase_it->second.scan_point);
                filtered_laser_scans_by_desired_angles_map.erase(erase_it);
            }
        }
        index_to_filter_vec.clear();
        current_index = compare_index;
    }
}

void LaserType::filterPointsByDistanceToNeighbors()
{
    int current_index = 0;
    int compare_index;
    auto start_it = filtered_laser_scans_by_desired_angles_map.begin();
    auto current_it = start_it;
    auto compare_it = start_it;
    while (current_index < filtered_laser_scans_by_desired_angles_map.size() -1 && !filtered_laser_scans_by_desired_angles_map.empty())
    {
        current_it = start_it;
        std::advance(current_it, current_index);
        LaserScanType current_laser_scan = current_it->second;
        compare_index = std::min(current_index + 1, (int)filtered_laser_scans_by_desired_angles_map.size() - 1); //try to fix it
        compare_index = std::max(compare_index, 1); //try to fix it
        compare_it = start_it;
        std::advance(compare_it, compare_index);
        LaserScanType next_laser_scan = compare_it->second;
        double distance_between_neightbors_points = getEuclideanDistance_ros(current_laser_scan.scan_point, next_laser_scan.scan_point);
        if (distance_between_neightbors_points > 1.0)
        {
            filtered_laser_scans_by_desired_angles_map.erase(compare_it);
            filtered_laser_scans_by_desired_angles_map.erase(current_it);
        }
        current_index = compare_index;
    }
}

void LaserType::UpdateObjectMembers(double start_angle, double end_angle)
{
    calculateRawLaserMapByAnglesFromRobot(start_angle, end_angle);
    calculateFilteredLaserMapByAnglesFromRobot();
    return;
}

std::vector <double> LaserType::FilterNanValuesFromVec(std::vector <double> original_vector)
{
    std::vector <double> output_vector;
    output_vector.clear();
    for (int i=0; i<original_vector.size(); i++)
    {
        double current_value = original_vector.at(i);
        if (std::isnan(current_value) || current_value == 0)
        {
            continue;
        }
        else
        {
            output_vector.push_back(current_value);
        }
    }
    return output_vector;
}

geometry_msgs::Point LaserType::getLaserPointFromAngleAndRange(double angle_from_robot, double range_from_robot)
{
    geometry_msgs::Point point;
    tf::Vector3 robot_to_lidar_scan = tf::Vector3(cos(angle_from_robot), sin(angle_from_robot), 0).normalized()*range_from_robot;
    point.x = robot_to_lidar_scan.getX();
    point.y = robot_to_lidar_scan.getY();
    point.z = 0;
    return point;
}

std::vector <double> LaserType::ConvertRawLaserRangesToLaserTypeVec(const std::vector <float> original_vector)
{
    std::vector <double> laser_ranges;
    laser_ranges.clear();
    for (int i=0; i<original_vector.size(); i++)
    {
        laser_ranges.push_back(original_vector[i]);
    }
    return laser_ranges;
}

int LaserType::ConvertAngleFromRobotToLaserIndex(double angle_from_robot)
{
    angle_from_robot = wrapAngleTo2Pi(angle_from_robot);
    double angle_factor =  angle_from_robot / D_2_PI; //360 deg
    int index = (int)(original_laser_scan_vec.size() * angle_factor);
    return index;
}

double LaserType::ConvertLaserIndexToAngleFromRobot(int laser_index)
{
    double angle_from_robot = ((double)laser_index * D_2_PI) /((double)original_laser_scan_vec.size());
    angle_from_robot = angle_from_robot;
    return angle_from_robot;
}

void LaserType::PrintLaserFilteredPoints()
{
    for( const auto& n : filtered_laser_scans_by_desired_angles_map )
    {
        double angle_from_robot = n.first;
        geometry_msgs::Point laser_point = n.second.scan_point;
    }
}
