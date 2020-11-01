#include<iostream>
#include<math.h>
#include<vector>
#include<algorithm>
#include<map>

using namespace std;

// �ĸ��̶���֮�乲���������룬�������һ����ϣ����
map<int, double> distance{ {1, 1.0}, {2, 2.0}, {3, 3.0}, {4, 4.0}, {5, 5.0}, {6, 6.0} };

// ȷ���ĸ���Ĺ�ϵ��Ȼ��������ϵ
// ���룺 1.�̶������������� 2.��NDI��ȡ��ʵʱ����, 3.����õ���Tools����ϵ�µ�����ֵ
// ԭ����NDI��ȡ��ʵʱ���꣬�����ĸ���֮��Ĺ̶����룬ȷ���ĸ���Ĺ�ϵ
//       Ȼ��������ϵ������NDI�豸�õ��ĸ�������ת��ΪTools����ϵ����(��Ϊ�ĸ���λ���ǹ̶��ģ�ֻ�账��һ�μ���)
void CorrespondenceAndBuild(const map<int, double>& distance, const vector<vector<double>>& NDICoordinates, vector<vector<double>>& ToolsCoordinates)
{
	// ����NDI���������֮��ľ���
	double dist;
	vector<int> correspond_index;          // ��Ÿ������붨���֮��Ķ�Ӧ���
	for (int i = 0; i < NDICoordinates.size(); i++)
	{
		int sum_dist_index = 0;       // 1: 1+2+3=6, 2:1+4+6=11��3��2+4+5=11�� 4��3+5+6=14
		vector<int> dist_indexs;     // ���ڸ����жϣ���dist_index�ĺ���11��ʱ���ж��ǵڶ�������������
		for (int j = 0; j < NDICoordinates.size(); j++)
		{
			if (i != j)
			{
				dist = sqrt(pow((NDICoordinates[i][0] - NDICoordinates[j][0]), 2) + pow((NDICoordinates[i][1] - NDICoordinates[j][1]), 2) + pow((NDICoordinates[i][2] - NDICoordinates[j][2]), 2));  // �������
				map<int, double> distcopy(distance);
				// ������ĵ���������Ԥ����ľ������������map���Խ��д�С����
				for (auto iter = distcopy.begin(); iter != distcopy.end(); iter++)
				{
					iter->second = abs(iter->second - dist);
				}
				// �������������
				auto iter = distcopy.begin();
				sum_dist_index += iter->first;
				dist_indexs.push_back(iter->first);

			}

		}
		if (sum_dist_index == 6)             // ��Ӧ��һ����
			correspond_index.push_back(1);
		else if (sum_dist_index == 14)        // ��Ӧ���ĸ���
			correspond_index.push_back(4);
		else                  // sum_dist_index==14ʱ��Ȼ���ٸ�������жϵڶ������������
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