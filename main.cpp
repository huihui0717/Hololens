#include<iostream>
#include<math.h>
#include<vector>
#include<algorithm>
#include<map>

using namespace std;

// 四个固定点之间共有六个距离，将其放入一个哈希表中
map<int, double> distance{ {1, 1.0}, {2, 2.0}, {3, 3.0}, {4, 4.0}, {5, 5.0}, {6, 6.0} };

// 确定四个点的关系，然后建立坐标系
// 输入： 1.固定距离间的向量， 2.从NDI读取的实时坐标, 3.程序得到的Tools坐标系下的坐标值
// 原理：从NDI读取的实时坐标，根据四个点之间的固定距离，确定四个点的关系
//       然后建立坐标系，将从NDI设备得到四个点坐标转化为Tools坐标系坐标(因为四个球位置是固定的，只需处理一次即可)
void CorrespondenceAndBuild(const map<int, double>& distance, const vector<vector<double>>& NDICoordinates, vector<vector<double>>& ToolsCoordinates)
{
	// 计算NDI坐标各坐标之间的距离
	double dist;
	vector<int> correspond_index;          // 存放各点与与定义点之间的对应序号
	for (int i = 0; i < NDICoordinates.size(); i++)
	{
		int sum_dist_index = 0;       // 1: 1+2+3=6, 2:1+4+6=11，3：2+4+5=11， 4：3+5+6=14
		vector<int> dist_indexs;     // 用于辅助判断，在dist_index的和是11的时候判断是第二个点或第三个点
		for (int j = 0; j < NDICoordinates.size(); j++)
		{
			if (i != j)
			{
				dist = sqrt(pow((NDICoordinates[i][0] - NDICoordinates[j][0]), 2) + pow((NDICoordinates[i][1] - NDICoordinates[j][1]), 2) + pow((NDICoordinates[i][2] - NDICoordinates[j][2]), 2));  // 计算距离
				map<int, double> distcopy(distance);
				// 将计算的点与点距离与预定义的距离做差，并利用map特性进行大小排序
				for (auto iter = distcopy.begin(); iter != distcopy.end(); iter++)
				{
					iter->second = abs(iter->second - dist);
				}
				// 计算距离索引和
				auto iter = distcopy.begin();
				sum_dist_index += iter->first;
				dist_indexs.push_back(iter->first);

			}

		}
		if (sum_dist_index == 6)             // 对应第一个点
			correspond_index.push_back(1);
		else if (sum_dist_index == 14)        // 对应第四个点
			correspond_index.push_back(4);
		else                  // sum_dist_index==14时，然后再根据序号判断第二个或第三个点
		{
			sort(dist_indexs.begin(), dist_indexs.end());
			if (dist_indexs[0] == 1)
				correspond_index.push_back(2);
			else
				correspond_index.push_back(3);
		}
	}

	
}








int main()
{



	return 0;
}