#include<iostream>
#include<math.h>
#include<vector>
#include<algorithm>
#include<map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;




// 四个固定点之间共有六个距离，将其放入一个哈希表中
//map<int, double> distance{ {1, 30.02}, {2, 59.53}, {3, 115}, {4, 30.02}, {5, 59.53}, {6, 88.02} };

// 确定四个点的关系，然后建立坐标系
// 输入： 1.固定距离间的向量， 2.从NDI读取的实时坐标, 3.程序得到的Tools坐标系下的坐标值
// 原理：从NDI读取的实时坐标，根据四个点之间的固定距离，确定四个点的关系
//       然后建立坐标系，将从NDI设备得到四个点坐标转化为Tools坐标系坐标(因为四个球位置是固定的，只需处理一次即可)
void CorrespondenceAndBuild(const vector<vector<double>>& NDICoordinates, vector<vector<double>>& ToolsCoordinates, vector<int>& correspond_index)
{ 
	//********************************************* 第一步找点的对应位置关系 ************************************************
	// 四个固定点之间共有六个距离，将其放入一个哈希表中
	map<double, int> distance{ {30.02, 1}, {59.53, 2}, {115, 3}, {30.02, 4}, {59.53, 5}, {88.02, 6} };
	// 计算NDI坐标各坐标之间的距离
	double dist;
	//vector<int> correspond_index;          // 存放各点与与定义点之间的对应序号
	for (int i = 0; i < NDICoordinates.size(); i++)
	{
		//int sum_dist_index = 0;       // 1: 1+2+3=6, 2:1+4+6=11，3：2+4+5=11， 4：3+5+6=14   存在歧义不能使用
		vector<int> dist_indexs;     // 用于辅助判断，在dist_index的和是11的时候判断是第二个点或第三个点
		for (int j = 0; j < NDICoordinates.size(); j++)
		{
			if (i != j)
			{
				dist = sqrt(pow((NDICoordinates[i][0] - NDICoordinates[j][0]), 2) + pow((NDICoordinates[i][1] - NDICoordinates[j][1]), 2) + pow((NDICoordinates[i][2] - NDICoordinates[j][2]), 2));  // 计算距离
				// 测试
				cout<< "第" << i << "与第" << j << "之间的距离：" << dist << endl;
				map<double, int> diff_dist;

				// 将计算的点与点距离与预定义的距离做差，并利用map特性进行大小排序
				for (auto iter_dist = distance.begin(); iter_dist != distance.end(); iter_dist++)
				{
					diff_dist[abs(iter_dist->first - dist)] = iter_dist->second;
				}
				// 计算距离索引和
				auto iter = diff_dist.begin();
				//sum_dist_index += iter->first;

				dist_indexs.push_back(iter->second);

			}

		}
		sort(dist_indexs.begin(), dist_indexs.end());
		// 测试
		for (int i = 0; i < dist_indexs.size(); i++)
			cout << "编号：" << dist_indexs[i] << endl;
		if (dist_indexs[2] == 6)
		{
			if (dist_indexs[0] == 3 || dist_indexs[1] == 3)
				correspond_index.push_back(4);
			else
				correspond_index.push_back(2);
		}
		else
		{
			if (dist_indexs[0] == 3 || dist_indexs[1] == 3 || dist_indexs[2] == 3)
				correspond_index.push_back(1);
			else
				correspond_index.push_back(3);
		}
	}

	// 测试
	cout << "第一个位置对应的编号：" << correspond_index[0] << endl;
	cout << "第二个位置对应的编号：" << correspond_index[1] << endl;
	cout << "第三个位置对应的编号：" << correspond_index[2] << endl;
	cout << "第四个位置对应的编号：" << correspond_index[3] << endl;

	map<int, int> correspond_map;
	for (int i = 0; i < correspond_index.size(); i++)
	{
		correspond_map[correspond_index[i]] = i;
	}
	
	// 按顺序依次将NDI坐标值放入数组中
	vector<vector<double>> NDI_Sorted_Coordinates;
	for (auto iter02 = correspond_map.begin(); iter02 != correspond_map.end(); iter02++)
	{
		NDI_Sorted_Coordinates.push_back(NDICoordinates[iter02->second]);
	}

	// 建立Tools坐标系
	// 先找共面(当作XY平面)的两条直线
	vector<double> XY_ax_01{NDI_Sorted_Coordinates[2][0] - NDI_Sorted_Coordinates[0][0], NDI_Sorted_Coordinates[2][1] - NDI_Sorted_Coordinates[0][1], NDI_Sorted_Coordinates[2][2] - NDI_Sorted_Coordinates[0][2] };           // 第三个点与第一个点构成的向量
	vector<double> XY_ax_02{ NDI_Sorted_Coordinates[2][0] - NDI_Sorted_Coordinates[3][0], NDI_Sorted_Coordinates[2][1] - NDI_Sorted_Coordinates[3][1], NDI_Sorted_Coordinates[2][2] - NDI_Sorted_Coordinates[3][2] };           // 第三个点与第4个点构成的向量
	// 然后找一个向量与上述共面的两条直线垂直(Z轴)（叉乘）
	vector<double> Z_ax{XY_ax_01[1]*XY_ax_02[2] - XY_ax_01[2]*XY_ax_02[1],
	XY_ax_01[2] * XY_ax_02[0] - XY_ax_01[0] * XY_ax_02[2], XY_ax_01[0] * XY_ax_02[1] - XY_ax_01[1] * XY_ax_02[0] };
	// 找X轴
	vector<double> X_ax{XY_ax_01};  
	// 现在已经找到了X轴与Z轴，然后再利用叉乘得到Y轴
	// 先单位化
	double X_S_SUM = sqrt(pow(X_ax[0], 2) + pow(X_ax[1], 2) + pow(X_ax[2], 2));
	for (int xi = 0; xi < 3; xi++)
		X_ax[xi] = X_ax[xi] / X_S_SUM;
	double Z_S_SUM = sqrt(pow(Z_ax[0], 2) + pow(Z_ax[1], 2) + pow(Z_ax[2], 2));
	for (int zi = 0; zi < 3; zi++)
		Z_ax[zi] = Z_ax[zi] / Z_S_SUM;
	// 叉乘得到Y轴坐标
	vector<double> Y_ax{ X_ax[1] * Z_ax[2] - X_ax[2] * Z_ax[1],
	X_ax[2] * Z_ax[0] - X_ax[0] * Z_ax[2], X_ax[0] * Z_ax[1] - X_ax[1] * Z_ax[0] };
	// 单位化
	double Y_S_SUM = sqrt(pow(Y_ax[0], 2) + pow(Y_ax[1], 2) + pow(Y_ax[2], 2));
	for (int yi = 0; yi < 3; yi++)
		Y_ax[yi] = Y_ax[yi] / Y_S_SUM;

	// 构建旋转平移矩阵
	vector<vector<double>> ToolstoNDI_TR{ X_ax, Y_ax, Z_ax, NDI_Sorted_Coordinates[2] };

	// 测试
	cout << "    " << ToolstoNDI_TR[0][0] << "    " << ToolstoNDI_TR[0][1] << "    " << ToolstoNDI_TR[0][2] << endl;
	cout << "    " << ToolstoNDI_TR[1][0] << "    " << ToolstoNDI_TR[1][1] << "    " << ToolstoNDI_TR[1][2] << endl;
	cout << "    " << ToolstoNDI_TR[2][0] << "    " << ToolstoNDI_TR[2][1] << "    " << ToolstoNDI_TR[2][2] << endl;
	cout << "    " << ToolstoNDI_TR[3][0] << "    " << ToolstoNDI_TR[3][1] << "    " << ToolstoNDI_TR[3][2] << endl;

	// ******************************************** 求解NDI与Tools之间的RT矩阵 ****************************************
	// 建立Tools坐标系
	// 以第三个点为原点，第三个点和第一点组成X轴，与 X 轴垂直的轴设为 Y 轴，根据右手准则构建Z轴
	// 其中第一点和第三个点的左边在Tools坐标系的位置已知，第二个点可以根据几何关系算出左边位置
	// 然后根据PnP算法算出两个坐标系之间的位姿
	//vector<float> Three{ 0.0, 0.0, 0.0 };        // 第三个点(原点)
	//vector<float> One{ 59.53, 0, 0 };            // 第一点在Tools下的坐标(已知)
	//vector<float> Two{ 29.76, -3.94, 0.0 };      // 第二点(可以根据几何关系算出)

	// 计算错误
	/*
	vector<Point3d> pts_3d;
	vector<Point2d> pts_2d;

	pts_2d.push_back(Point2d(59.53, 0));            
	pts_2d.push_back(Point2d(29.76, -3.94));
	pts_2d.push_back(Point2d(0.0, 0.0));
	pts_2d.push_back(Point2d(-51.54, 29.79));

	// 将前三个对应点放入 pts_3d
	for (auto iter1 = correspond_map.begin(); iter1 != correspond_map.end(); iter1++)
	{
		pts_3d.push_back(Point3d(NDICoordinates[iter1->second][0], NDICoordinates[iter1->second][1], NDICoordinates[iter1->second][2]));
	}

	Mat K = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     // 模拟相机参数

	Mat r, t;

	solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法

	Mat R;
	cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵

	cout << "R=" << endl << R << endl;
	cout << "t=" << endl << t << endl;

	// 验证
	vector<Point3d> test_pts_3d;
	*/



}




int main()
{
	vector<vector<double>> data1{ {-152.38687133789062,-414.9403076171875,-1809.3543701171875},
	{-153.75457763671875,-469.5207214355469,-1785.650390625},{-144.6283721923828,-492.3905944824219,-1768.420654296875},
	{-129.71180725097656,-510.6588134765625,-1750.1781005859375} };

	vector<vector<double>> data2{ {-99.11727142333984,-399.4183044433594,-1762.468505859375},
	{-61.08290481567383,-394.8155517578125,-1717.1427001953125}, {-40.60084915161133,-382.1108703613281,-1699.4599609375 },
	{-20.29187774658203,-363.5483703613281,-1686.89648437} };

	vector<vector<double>> data3{ {-17.180805206298828,-141.00416564941406,-1758.3082275390625},
	{-4.813007354736328,-157.35777282714844,-1736.5223388671875},{4.578853607177734,-179.47300720214844,-1718.52099609375},
	{10.813793182373047,-235.3511505126953,-1698.754638671875} };

	vector<vector<double>> data4{ {-3.167431592941284,-63.56769943237305,-1769.4393310546875},
	{9.057886123657227,-79.88369750976562,-1746.82275390625},{18.462421417236328,-102.10611724853516,-1729.0760498046875},
	{24.722509384155273,-158.11813354492188,-1709.818603515625} };

	vector<vector<double>> data5{ {-430.1352233886719,-89.00432586669922,-2215.77587890625},
	{-405.5355224609375,-36.878116607666016,-2201.065185546875},{-398.7040710449219,-7.743083953857422,-2203.693603515625},
	{-395.8187255859375,20.64977264404297,-2212.70458984375} };
	vector<vector<double>> data6{ {-175.40179443359375,-117.23219299316406,-1937.8348388671875},
	{-150.6659393310547,-65.19169616699219,-1922.7894287109375},{-143.68417358398438,-36.092098236083984,-1925.0963134765625},
	{-140.5762481689453,-7.694615840911865,-1934.1260986328125} };



	vector<vector<double>> ToolsCoordinates1;
	vector<int> select_index1;

	vector<vector<double>> ToolsCoordinates2;
	vector<int> select_index2;

	vector<vector<double>> ToolsCoordinates3;
	vector<int> select_index3;

	vector<vector<double>> ToolsCoordinates4;
	vector<int> select_index4;

	vector<vector<double>> ToolsCoordinates5;
	vector<int> select_index5;

	vector<vector<double>> ToolsCoordinates6;
	vector<int> select_index6;

	CorrespondenceAndBuild(data1, ToolsCoordinates1, select_index1);
	CorrespondenceAndBuild(data2, ToolsCoordinates2, select_index2);
	CorrespondenceAndBuild(data3, ToolsCoordinates3, select_index3);
	CorrespondenceAndBuild(data4, ToolsCoordinates4, select_index4);
	CorrespondenceAndBuild(data5, ToolsCoordinates5, select_index5);
	CorrespondenceAndBuild(data6, ToolsCoordinates6, select_index6);


	cout << "按任意键继续……";
	cin.clear();
	cin.sync();
	cin.get();

	return 0;
}