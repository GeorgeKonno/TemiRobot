#include <opencv/cv.h>
#include <iostream>
#include <functional>
#include <unordered_map>
#include <queue>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <string>
#include <vector>
#include <sstream>
#include<fstream>

using namespace std;

#include <boost/unordered_set.hpp>
using namespace boost;

template <typename T>

class pvec_point
{
public:
     //Constructors
     pvec_point(){
         this->X = std::numeric_limits<T>::infinity();
         this->Y = std::numeric_limits<T>::infinity();
     }
     pvec_point(T x, T y){
         this->X = x;
         this->Y = y;
     }
     pvec_point(const pvec_point& another){
         X = another.X;
         Y = another.Y;
     }
     T X,Y;
     //operators:
     bool operator==(const pvec_point& cur) const //The last const is  telling that the function isn't changing the values of both structs
     {
         return (cur.X==X && cur.Y==Y);
     }
     bool operator!=(const pvec_point& cur) const //The last const is  telling that the function isn't changing the values of both structs
     {
         return (cur.X!=X || cur.Y!=Y);
     }
     bool operator<(const pvec_point& cur) const //The last const is  telling that the function isn't changing the values of both structs
     {
        if(cur.X == X)//in case equal order by Y
            return Y < cur.Y;
        return X < cur.X;
     }
     pvec_point operator+(const pvec_point &P) const
     {
         pvec_point NewP;
         NewP.X = X+P.X;
         NewP.Y = Y+P.Y;
         return NewP;
     }
     pvec_point operator-(const pvec_point &P) const
     {
         pvec_point NewP;
         NewP.X = X-P.X;
         NewP.Y = Y-P.Y;
         return NewP;
     }
     pvec_point& operator = (const pvec_point &P)
     {
         X = P.X;
         Y = P.Y;
         return *this;
     }

     std::string print(char Seperator = ',')
     {
         std::stringstream buffer;
         buffer<<"("<<this->X<<Seperator<<this->Y<<")";
         return buffer.str();
     }
     //converters
     static pvec_point convert_cvpoint_to_Point(const cv::Point &P)
     {
         return pvec_point(P.x,P.y);
     }
     static pvec_point convert_cvpoint_to_Point_swapped(const cv::Point &P)
     {
         return pvec_point(P.y,P.x);
     }
     bool rounding_and_equalization(pvec_point P)
     {
         return (((double)round(P.X) == (double)round(this->X)) && ((double)round(P.Y) == (double)round(this->Y)));
     }
     //~~~~~~~~ Getters ~~~~~~~~~//
     inline T getX() const { return this->X; }
     inline T getY() const { return this->Y; }
     inline bool is_valid() { return (this->X == std::numeric_limits<T>::infinity()) && (this->Y == std::numeric_limits<T>::infinity()); }
     bool is_null_cv_coordinate() { return this->X==-1 && this->Y==-1;}
     //~~~~~~~~ Setters ~~~~~~~~~//
     inline pvec_point swap_xy() { return pvec_point(this->Y,this->X);} /// swap between x and y just for fun
     inline void set_null_cv_coordinate() { this->X=-1; this->Y=-1; }
};

#define IntPoint2D pvec_point<int>

bool IntPoint2D_Equal(IntPoint2D A, IntPoint2D B)
{
    if (A.X==B.X && A.Y == B.Y)
    {
        return true;
    }
    return false;
}

class Node
{ 
private:
    IntPoint2D coordinates_;
    double value_;
public:
    Node(const IntPoint2D& coordinates, double value) : coordinates_(coordinates), value_(value){}
    Node(const IntPoint2D& coordinates) : coordinates_(coordinates), value_(0.0){}
    Node(int x, int y, double value) : coordinates_(x,y), value_(value) {}
    Node() = default;
    bool operator==(const Node& b) const {return IntPoint2D_Equal(coordinates_, b.getCoordinates());}
    bool operator>(const Node& b) const {return value_ > b.getValue();}
    bool operator<(const Node& b) const {return value_ < b.getValue();}
    void setValue(double value) {value_ = value; return;}
    double getValue() const {return value_;}
    IntPoint2D getCoordinates() const {return coordinates_;}
};

struct NodeHasher
{
    std::size_t operator()(const Node& n) const
    {
        IntPoint2D pixel = n.getCoordinates();
        return boost::hash<std::pair<int,int> >()(std::make_pair(pixel.getX(),pixel.getY()));
    }
};

inline double euclideanDistanceBetween(const IntPoint2D& a, const IntPoint2D& b)
{
    double dx = a.getX() - b.getX(), dy = a.getY() - b.getY();
    return sqrt(dx * dx + dy * dy);
}

inline double weightBetween_g(const Node &a, const Node &b)
{
    return euclideanDistanceBetween(a.getCoordinates(), b.getCoordinates());
}

inline double weightBetween(const Node &a, const Node &b,cv::Mat obstacle_distance_map_,cv::Mat visited_pixels_map_,cv::Mat front_cone_map_) /// weight(a,b) != weight(b,a)
{
    //cout<<"start"<<endl;
    double euclidean_dist = weightBetween_g(a, b) * 0.05;
    IntPoint2D point_b = b.getCoordinates();
    cv::Point tmp_point(point_b.getX(), point_b.getY());
    //cout<<"end"<<endl;
    //cout<<obstacle_distance_map_<<endl;
    double height_at_b = obstacle_distance_map_.at<double>(tmp_point);
    double point_was_visited =
        (visited_pixels_map_.at<uchar>(tmp_point) == 0)
        ? (0.0)
        : (0.5); /// assuming that redmap is given here and that a visited pixel is a free one
    double front_cone_point =
        (front_cone_map_.at<uchar>(tmp_point) == 0)
        ? (0.0)
        : (0.5); /// assuming that redmap is given here and that a visited pixel is a free one
    //cout<<"end"<<endl;
    return euclidean_dist +
        5.0 * height_at_b +
        100.0 * point_was_visited +
        5.0 * front_cone_point;
}

std::vector<IntPoint2D> reconstructPath(const std::unordered_map<Node, Node, NodeHasher> &backpointer,const Node &goal,const Node &start)
{
      std::vector<IntPoint2D> path;
      Node current_node(goal);
      path.reserve(backpointer.size() + 1);
      while (current_node.getCoordinates() != start.getCoordinates())
      {
          path.push_back(current_node.getCoordinates());
          current_node = backpointer.at(current_node); /// get the next pixel on the path
      }   path.push_back(start.getCoordinates());
      std::reverse(path.begin(), path.end());
      //cout<<"path is: "<<endl;
      //cout <<"from ";
      //for(std::vector<IntPoint2D>::iterator iter = path.begin(); iter != path.end(); iter++)
      //{
          //cout <<"("<<iter->X <<","<< iter->Y<<")"<<" to ";
      //}  
      //cout <<" end!!!"<<endl;
      return path;
}

std::vector<IntPoint2D> getNeighbours(const Node &node,cv::Mat obstacles_map_) 
{
    const IntPoint2D& pixel = node.getCoordinates();
    static const std::array<IntPoint2D, 8> all_neighbors{
      IntPoint2D(-1,-1), IntPoint2D(-1,0), IntPoint2D(-1,1),
          IntPoint2D(0,-1),                   IntPoint2D(0,1),
          IntPoint2D(1,-1), IntPoint2D(1,0), IntPoint2D(1,1)};
    const int& rows = obstacles_map_.rows;
    const int& cols = obstacles_map_.cols;
    std::vector<IntPoint2D> valid_neighbours;
    valid_neighbours.reserve(8);
    for (IntPoint2D const& delta : all_neighbors)
    {
        IntPoint2D cur_pixel = pixel + delta;
        if ((cur_pixel.X < cols) && (cur_pixel.Y < rows) &&
                (cur_pixel.X >= 0)  && (cur_pixel.Y >= 0))
        {
            cv::Point tmp_pixel(cur_pixel.X,cur_pixel.Y);
            if (obstacles_map_.at<uchar>(tmp_pixel) == 0)
            {
                valid_neighbours.push_back(cur_pixel);
            }
        }
    }
    return valid_neighbours;
}
/*
ARA  GOGOGOGOGOGOGOGOGOGO
*/
/*
ARA  GOGOGOGOGOGOGOGOGOGO
*/
/*
ARA  GOGOGOGOGOGOGOGOGOGO
*/
double f_value(
    std::unordered_map<Node, double, NodeHasher>& g, 
    Node s,double e, Node& goal,cv::Mat& obstacle_distance_map,cv::Mat& visited_pixels_map,cv::Mat& front_cone_map)
{
    return  g.at(s)+ e * 800 * weightBetween(s,goal,obstacle_distance_map,visited_pixels_map,front_cone_map);
}

double min_value(
    std::unordered_map<Node, double, NodeHasher>& g, 
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>>& q,
Node& goal,cv::Mat& obstacle_distance_map,cv::Mat& visited_pixels_map,cv::Mat& front_cone_map)
{
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>> tmp;
    double min_node_cost = 99999999;
    while(!q.empty())
    {
        Node a(q.top());
        q.pop();
        if(min_node_cost > f_value(g,a,1,goal,obstacle_distance_map,visited_pixels_map,front_cone_map))
        {
            min_node_cost = f_value(g,a,1,goal,obstacle_distance_map,visited_pixels_map,front_cone_map);
        }
        tmp.push(a);
    }
    q = tmp;
    return min_node_cost;
}

void reset_q_value(
    std::unordered_map<Node, double, NodeHasher>& g, 
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>>& q,double& e,
Node& goal,cv::Mat& obstacle_distance_map,cv::Mat& visited_pixels_map,cv::Mat& front_cone_map)
{
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>> tmp;
    while(!q.empty())
    {
        Node a(q.top());
        q.pop();
        a.setValue(f_value(g,a,e,goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
        tmp.push(a);
    }
    q = tmp;
}

double computerPath(
    std::unordered_map<Node, double, NodeHasher>& g, 
    std::unordered_map<Node, Node, NodeHasher>& backpointer, 
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>>& q, 
    std::unordered_map<Node, double, NodeHasher>& closed,
    std::unordered_map<Node, double, NodeHasher>& incons,
    double& e,
    Node& goal,cv::Mat& obstacle_distance_map,cv::Mat& visited_pixels_map,cv::Mat& front_cone_map,cv::Mat& obstacles_map
)
{
    cout<<"q size is:"<<q.size()<<endl;
    Node current_node(q.top());
    while(!q.empty())
    {
        Node current_node(q.top());
        q.pop();
        closed.insert({current_node,0.0});
        if(current_node == goal){
            //cout<<"found"<<endl;
            break;
        }
        std::vector<IntPoint2D> neighbours = getNeighbours(current_node,obstacles_map);
        for (auto const& neighbour : neighbours)
        {
            Node child(neighbour);
            if(g.find(child) == g.end())
            {
                g[child] = 99999999;
            }
            if (g.at(child) >
            g.at(current_node) + weightBetween(current_node,child,obstacle_distance_map,visited_pixels_map,front_cone_map)) 
            {
                g[child] = g.at(current_node) + weightBetween(current_node,child,obstacle_distance_map,visited_pixels_map,front_cone_map);
                if(closed.find(child) == closed.end())
                {
                    child.setValue(f_value(g,child,e,goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
                    q.push(child);
                }
                else
                {
                    incons[child] = 0;
                }
                backpointer[child] = current_node;
            }
        }
    }
    cout<<"cost is: "<<g.at(goal)*0.05<<endl;
    return g.at(goal)*0.05;
}

void insert(std::unordered_map<Node, double, NodeHasher>& incons,std::priority_queue<Node, std::deque<Node>, std::greater<Node>>& q)
{
    auto iter = incons.begin();
    while(iter != incons.end())
    {
        q.push(iter->first);
        ++iter;
    }
}

std::vector<IntPoint2D> calculatePath(IntPoint2D start_pixel_,IntPoint2D goal_pixel_,cv::Mat obstacles_map,cv::Mat obstacle_distance_map,cv::Mat visited_pixels_map,cv::Mat front_cone_map)
{
    std::unordered_map<Node, double, NodeHasher> g; 
    std::unordered_map<Node, Node, NodeHasher> backpointer; 
    std::priority_queue<Node, std::deque<Node>, std::greater<Node>> q; 
    std::unordered_map<Node, double, NodeHasher> closed;
    std::unordered_map<Node, double, NodeHasher> incons;
    std::vector<IntPoint2D> path;
    double e = 2.5;
    Node goal(goal_pixel_), start(start_pixel_);
    g.reserve(obstacles_map.total());
    g.insert({start,0.0});
    g.insert({goal,99999999});
    start.setValue(f_value(g,start,e, goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
    q.push(start);
    computerPath(g,backpointer,q,closed,incons,e,goal,obstacle_distance_map,visited_pixels_map,front_cone_map,obstacles_map);
    insert(incons,q);
    //e = std::min(e,g[goal]/min_value(g,q,goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
    while(e > 1)
    {   
        cout<<"e is: "<<e<<endl;
        e = e - 0.5;
        reset_q_value(g,q,e,goal,obstacle_distance_map,visited_pixels_map,front_cone_map);
        closed.clear();
        computerPath(g,backpointer,q,closed,incons,e,goal,obstacle_distance_map,visited_pixels_map,front_cone_map,obstacles_map);
        insert(incons,q);
        //e = std::min(e,g[goal]/min_value(g,q,goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
        path = reconstructPath(backpointer, goal, start);
        //cout<<"cost is: "<<g.at(goal)*0.05<<endl;
    }
    path = reconstructPath(backpointer, goal, start);
    //cout<<"cost is: "<<g.at(goal)*0.05<<endl;
    return path;

}
/*
ARA  ENENENENENENENENENEN
*/
/*
ARA  ENENENENENENENENENEN
*/
/*
ARA  ENENENENENENENENENEN
*/

// std::vector<IntPoint2D> calculatePath(IntPoint2D start_pixel_,IntPoint2D goal_pixel_,cv::Mat obstacles_map,cv::Mat obstacle_distance_map,cv::Mat visited_pixels_map,cv::Mat front_cone_map)
// {
//     std::unordered_map<Node, double, NodeHasher> g; /// maps between a Node to it's distance to the start
//     std::unordered_map<Node, Node, NodeHasher> backpointer; /// points back from node a to node b (goal->goal-1->goal-2->...->start)
//     std::priority_queue<Node, std::deque<Node>, std::greater<Node>> q; /// using std::greater so the smallest element is at the top.
//     //cout<<"ok1"<<endl;
//     g.reserve(obstacles_map.total()); 
//     //cout<<"ok2"<<endl;
//     Node goal(goal_pixel_), start(start_pixel_);
//     //cout<<"ok3"<<endl;
//     //cout<<obstacle_distance_map<<endl;
//     Node root(start_pixel_, weightBetween(start,goal,obstacle_distance_map,visited_pixels_map,front_cone_map));
//     //cout<<"ok4"<<endl;
//     q.push(root);
//     g.insert({root,0.0});
//     backpointer.insert({root,root}); /// if there is no path, just stay at the start
//     int count = 0;
//     while (!q.empty())
//     {
//         //cout<<g.size()<<endl;
//         count++;
//         //if(count > 10){
//             //return false;
//         //}
//         Node current_node(q.top());
//         q.pop();
//         //const IntPoint2D& pixel = current_node.getCoordinates();
//         //cout<<"xuanle"<<"("<<pixel.X<<","<<pixel.Y<<")"<<endl;
//         if (current_node == goal)
//         {
//             cout<<"兄弟我找了 "<<count<<" 次就找到了牛逼了啊"<<endl;
//             std::vector<IntPoint2D> path = reconstructPath(backpointer, goal, root);
//             cout<<"cost is: "<<g.at(current_node)*0.05<<endl;
//             return path;
//         }
//         std::vector<IntPoint2D> neighbours = getNeighbours(current_node,obstacles_map);
//         for (auto const& neighbour : neighbours)
//         {
//             Node child(neighbour);
//             double weight2start = g.at(current_node) + weightBetween(current_node,child,obstacle_distance_map,visited_pixels_map,front_cone_map);
//             double h = weightBetween(child,goal,obstacle_distance_map,visited_pixels_map,front_cone_map);
//             double f = weight2start + h;
//             //const IntPoint2D& pixel = child.getCoordinates();
//             //cout<<"("<<pixel.X<<","<<pixel.Y<<")"<<"***h is:"<<h<<"***w is:"<<weight2start<<"***sum is:"<<h+weight2start<<endl;
//             if ((g.find(child) == g.end()) || weight2start < g.at(child)) /// this node hasn't been visited or better path to child found
//             {
//                 child.setValue(f);
//                 g[child] = weight2start;
//                 q.push(child);
//                 backpointer[child] = current_node;
//             }
//         }
//     }
//     if(count == 1){
//         cout<<"兄弟这两个点没法走不算失败"<<endl;
//         std::vector<IntPoint2D> path;
//         return path;
//     }
//     cout<<"兄弟我找了 "<<count<<" 次没找到我觉得没有路"<<endl;
//     std::vector<IntPoint2D> path;
//     return path;
// }

#define FILE_PATH "/home/gliu/Desktop/pp_maps"

cv::Mat colorReduce(cv::Mat image){
    for(int i=0;i<image.rows;i++)
    {
        for(int j=0;j<image.cols;j++)
        {
            image.at<uchar>(i,j) = 255;
        }
    }
    return image;
}

int main(){
    cv::Mat o_d_map = cv::imread("../pp_maps/cost_maps/2016-02-11_19\:28\:38_287_718_573_320.png",0);
    cv::Mat v_p_map = cv::imread("../pp_maps/attraction_maps/2016-02-11_19\:28\:38_287_718_573_320.png",0);
    cv::Mat f_c_map = cv::imread("../pp_maps/cone_maps/2016-02-11_19\:28\:38_287_718_573_320.png",0);
    cv::Mat o_map = cv::imread("../pp_maps/regular_maps/2016-02-11_19\:28\:38_287_718_573_320.png",0);
    IntPoint2D start_pixel;
    IntPoint2D goal_pixel;
    start_pixel.X = static_cast<int>(287); start_pixel.Y = static_cast<int>(718);
    goal_pixel.X = static_cast<int>(573); goal_pixel.Y = static_cast<int>(320);
    // cout<<"start write file"<<endl;
    // ofstream out;
    // out.open("../point_set.log");//清空文件内容
    // out.close();
    // for(int i=0;i<o_map.rows;i++)
    // {
    //     for(int j=0;j<o_map.cols;j++)
    //     {
    //         if(o_map.at<uchar>(i,j) == 0){
    //             out.open("../point_set.log",ios::app);//app表示每次操作前均定位到文件末尾
    //             if(out.fail()){
    //                 cout<<"open error\n";
    //             }
    //             out<<i<<endl<<j<<endl;
    //             out.close();
    //         }
    //     }
    // }
    // cout<<"end write file"<<endl;
    /*
        cv::Mat o_map;
        cvtColor(imgGray, o_map, CV_GRAY2BGR);
        //cout<<o_d_map<<endl;
        Node end(goal_pixel), begin(start_pixel);
        const IntPoint2D& pixel1 = end.getCoordinates();
        const IntPoint2D& pixel2 = begin.getCoordinates();
        cv::Point tmp_pixel1(pixel1.X,pixel1.Y);
        cv::Point tmp_pixel2(pixel2.X,pixel2.Y);
        cv::circle(o_map, tmp_pixel1, 100, CV_RGB(0, 255, 0), 1, 8, 3);
        cv::circle(o_map, tmp_pixel2, 100, CV_RGB(0, 255, 0), 1, 8, 3);
        cv::imwrite("../1.png",o_map);
    */
    clock_t start,ends;
    start=clock();
    std::vector<IntPoint2D> result = calculatePath(start_pixel,goal_pixel,o_map,o_d_map,v_p_map,f_c_map);
    ends=clock();
    cout<<"use time is: "<<(ends-start)/1000000.000000000000000<<"s"<<endl;
    //cout<<"find path? "<<result<<endl;
    cv::imwrite("../1.png",o_map);
    cv::Mat new_map = colorReduce(o_map);
    for(std::vector<IntPoint2D>::iterator iter = result.end(); iter != result.begin(); iter--)
    {
        new_map.at<uchar>(iter->Y,iter->X) = 0;
        //cout <<"("<<iter->X <<","<< iter->Y<<")"<<" to ";
    }  
    cv::imwrite("../2.png",new_map);



    

    /*cout<<"kai shi sui ji ce shi"<<endl;
    o_map = cv::imread("../pp_maps/regular_maps/1.png",0);
    std::vector<int> X;
    std::vector<int> Y;
    string s;
    ifstream file("../point_set.log");
    cout<<"debug"<<endl;   
    int deal = 0;
    while(getline(file,s))
    {
        stringstream stream;
        int number;
        string text(s);
        stream << text;
        stream >> number ;
        stream.clear();
        if(deal%2 == 0){
            X.push_back(number);
        }
        else{
            Y.push_back(number);
        }
        deal = deal + 1;
        //cout<<s<<endl;
    }
    cout<<X.size()<<","<<Y.size()<<endl;
    file.close();
    cout<<endl<<endl;
    for (int i = 0; i < 10000; i++)
    {
        srand(i);
        auto random1 = (rand()%(X.size()))+0;
        srand(i+10000);
        auto random2 = (rand()%(X.size()))+0;
        cout<<random1<<" , "<<random2<<endl;
        //cout<<X.at(random1)<<endl;
        start_pixel.X = X.at(random1); start_pixel.Y = Y.at(random1);
        goal_pixel.X = X.at(random2); goal_pixel.Y = Y.at(random2);
        clock_t start,ends;
        start=clock();
        std::vector<IntPoint2D> result = calculatePath(start_pixel,goal_pixel,o_map,o_d_map,v_p_map,f_c_map);
        ends=clock();
        cout<<"use time is: "<<(ends-start)/1000000.000000000000000<<"s"<<endl;
        cout<<endl<<endl;
    }
    cout<<"jie shu sui ji ce shi"<<endl;*/
    




    return 0;
}
