#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
 
class CPoint
{
public:
    CPoint(int x,int y):X(x),Y(y),G(0),H(0),F(0),m_parentPoint(NULL){ };
    ~CPoint();
    int X,Y,G,H,F;
    CPoint * m_parentPoint;
    void CalF(){
        F=G+H;
    };
};
class  CAStar
{
    private:
	int m_array[12][12];
	static const int STEP = 10;
    static const int OBLIQUE = 14;
    
	typedef std::vector<CPoint*> POINTVEC;
	POINTVEC m_openVec;
	POINTVEC m_closeVec;
    public:
        CAStar(int array[][12])
        {
            for (int i=0;i<12;i++)
                for(int j=0;j<12;j++)
                    m_array[i][j]=array[i][j];
        }
        CPoint* GetMinFPoint()
        {
            int idx=0,valueF=-9999;
            for(int i=0; i < m_openVec.size(); ++i)
                {
                    if(m_openVec[i]->F < valueF)
                    {
                        valueF = m_openVec[i]->F;
                        idx = i;
                    }
                }
		return m_openVec[idx];
        }
 
	bool RemoveFromOpenVec(CPoint* point)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if((*it)->X == point->X && (*it)->Y == point->Y)
			{
				m_openVec.erase(it);
				return true;
			}
		}
		return false;
	}
	
	bool canReach(int x, int y)
	{
		return 0 == m_array[x][y];
	}
 
	bool IsAccessiblePoint(CPoint* point, int x, int y, bool isIgnoreCorner)
	{
		if(!canReach(x, y) || isInCloseVec(x, y))
			return false;
		else
		{
			//可到达的点
			if(abs(x - point->X) + abs(y - point->Y) == 1)    // 左右上下点
				return true;
			else
			{
				if(canReach(abs(x - 1), abs(y + 1)) || canReach(abs(x + 1), abs(y - 1))||canReach(abs(x - 1), abs(y - 1)) || canReach(abs(x + 1), abs(y + 1)))   // 对角点
					return true;
				else
					return isIgnoreCorner;   //墙的边角
			}
		}
	}
 
	std::vector<CPoint*> GetAdjacentPoints(CPoint* point, bool isIgnoreCorner)
	{
		POINTVEC adjacentPoints;
		for(int x = point->X-1; x <= point->X+1; ++x)
			for(int y = point->Y-1; y <= point->Y+1; ++y)
			{
				if(IsAccessiblePoint(point, x, y, isIgnoreCorner))
				{
					CPoint* tmpPoint = new CPoint(x, y);
					adjacentPoints.push_back(tmpPoint);
				}
			}
 
		return adjacentPoints;
	}
 
	bool isInOpenVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}
 
	bool isInCloseVec(int x, int y)
	{
		for(POINTVEC::iterator it = m_closeVec.begin(); it != m_closeVec.end(); ++it)
		{
			if((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}
	
	void RefreshPoint(CPoint* tmpStart, CPoint* point)
	{
		int valueG = CalcG(tmpStart, point);
		if(valueG < point->G)
		{
			point->m_parentPoint = tmpStart;
			point->G = valueG;
			point->CalF();
		}
	}
	
	void NotFoundPoint(CPoint* tmpStart, CPoint* end, CPoint* point)
	{
		point->m_parentPoint = tmpStart;
		point->G = CalcG(tmpStart, point);
		point->G = CalcH(end, point);
		point->CalF();
		m_openVec.push_back(point);
	}
	
	int CalcG(CPoint* start, CPoint* point)
	{
		int G = (abs(point->X - start->X) + abs(point->Y - start->Y)) == 2 ? STEP : OBLIQUE;
		int parentG = point->m_parentPoint != NULL ? point->m_parentPoint->G : 0;
		return G + parentG;
	}
	
	int CalcH(CPoint* end, CPoint* point)
	{
		int step = abs(point->X - end->X) + abs(point->Y - end->Y);
		return STEP * step;
	}
 
	// 搜索路径
	CPoint* FindPath(CPoint* start, CPoint* end, bool isIgnoreCorner)
	{
		m_openVec.push_back(start);
		while(0 != m_openVec.size())
		{
			CPoint* tmpStart = GetMinFPoint();   // 获取F值最小的点
			RemoveFromOpenVec(tmpStart);
			m_closeVec.push_back(tmpStart);
			
			POINTVEC adjacentPoints = GetAdjacentPoints(tmpStart, isIgnoreCorner);
			for(POINTVEC::iterator it=adjacentPoints.begin(); it != adjacentPoints.end(); ++it)
			{
				CPoint* point = *it;
				if(isInOpenVec(point->X, point->Y))   // 在开启列表
					RefreshPoint(tmpStart, point);
				//else if(inCloseVec(point))
				//{
				// 检查节点的g值，如果新计算得到的路径开销比该g值低，那么要重新打开该节点（即重新放入OPEN集）		
				//}
				else
					NotFoundPoint(tmpStart, end, point);
			}
			if(isInOpenVec(end->X, end->Y)) // 目标点已经在开启列表中
			{
				for(int i=0; i < m_openVec.size(); ++i)
				{
					if(end->X == m_openVec[i]->X && end->Y == m_openVec[i]->Y)
						return m_openVec[i];
				}
			}
		}
		return end;
	}
 
 
};
 
int main()
{
    int start_point_x=1;
    int start_point_y=1;
    int goal_point_x=5;
    int goal_point_y=2;
    int array[12][12] = 
    { //  0  1  2  3  4  5  6  7  8  9 10  11    
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},// 0 
    { 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1},// 1
	{ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},// 2
	{ 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},// 3
	{ 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},// 4
	{ 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1},// 5
	{ 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},// 6
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} // 7
    };
	
    CAStar* pAStar = new CAStar(array);
    if(array[start_point_x][start_point_y]||array[goal_point_x][goal_point_y])
    {
        cout<<"start point or goal point set error!!!"<<endl;
        return 0;
    }
    CPoint* start = new CPoint(start_point_x, start_point_y);
    CPoint* end = new CPoint(goal_point_x, goal_point_y);
    CPoint* point = pAStar->FindPath(start, end, false);
 
    Rect rect;
    Point left_up,right_bottom;
    
    Mat img(600,600,CV_8UC3,Scalar(255,255,255));
    namedWindow("Test"); 
    
    while(point != NULL)
    {
        left_up.x = point->Y*50;  //存储数组的列(point->Y)对应矩形的x轴，一个格大小50像素
        left_up.y = point->X*50;  
        right_bottom.x = left_up.x+50;  
        right_bottom.y = left_up.y+50;
        rectangle(img,left_up,right_bottom,Scalar(0,255,255),CV_FILLED,8,0);//path yellow(full)
        std::cout << "(" << point->X << "," << point->Y << ");" << std::endl;
        point = point->m_parentPoint;
        
    }
 
    for(int i=0;i<8;i++)
    {
        for(int j=0;j<12;j++)
        {   
            left_up.x = j*50; //存储数组的列(j)对应矩形的x轴
            left_up.y = i*50;  
            right_bottom.x = left_up.x+50;  
            right_bottom.y = left_up.y+50;
            if(array[i][j])
            {
                rectangle(img,left_up,right_bottom,Scalar(0,0,255),CV_FILLED,8,0);//obstacles red
            }
            else
            {
                if(i==start_point_x&&j==start_point_y)
                    rectangle(img,left_up,right_bottom,Scalar(255,0,0),CV_FILLED,8,0);//start point blue(full)
                else if(i==goal_point_x&&j==goal_point_y)
                    rectangle(img,left_up,right_bottom,Scalar(0,0,0),CV_FILLED,8,0);//goal point black(full)
                else
                    rectangle(img,left_up,right_bottom,Scalar(0,255,0),2,8,0);//free white content,green edge
            }    
        }
    }
 
    
    imshow("Test",img);   //窗口中显示图像 
	waitKey(0);
	return 0;
    
    
}

