//ͷ�ļ�һ�������Ķ��塢extern�����������ͺ���������const����
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <vector>
using namespace std;

//const������
const double P_SENCAR = 10;                             //Sencar�����������ʣ���λJ/S
const int SIMUL_CHRAGE_NODE = 4;                        //���ڵ������ʱ��Sencar�ﵽ�����
const double BATTERY_ENERGY = 1000;                     //������������,��λJ(������
const double MOBILE_CONSUMPTION = 30;                   //Sencar���ƶ�����J/s
const double SPEED = 5;                                 //Sencar���ƶ����ʵ�λm/s
const int N_NODE = 100;                                 //�����������нڵ������
const int M_SENCAR = 3;                                 //sencars����
const double LENGTH = 30;                               //����������ĳ���
const double WIDTH = 30;                                //����������Ŀ��
const double INF = 32767;
const double MAX_HOP_DIS = 1.5;                         //�м̳�絥��������
const double MAX_HOP = 1;                               //�м̳���ܾ�������
const int ROUND = 100 ;


//�ṹ�嶨�壺
class Point
{
public:
    int number;                 //�ڵ���ţ����� ��ͬ�ڵ��Ψһ����
    int group_number;           //�м̳��������
//    int father_node;            //��̽ڵ�����
//    int sum_children;           //�ӽڵ�ĸ��������ڼ���ڵ���ܺ�
    bool anchor;                //�ýڵ��Ƿ�Ϊê�ڵ�

    double coordinate_x;        //x����
    double coordinate_y;        //y����
    double current_enery;       //�������ĵ�ǰ����
    double efficiency;          //���Ч��
    double benifit;             //�Ըõ���Ϊê�������

    int symbol;                 //һ����ǣ�ê��ѡ���ʱ���Ƿ񱻱�����
    double charge_time;

public:
    Point():number(0),group_number(0),anchor(true),coordinate_x(0),coordinate_y(0),
    current_enery(BATTERY_ENERGY),efficiency(1),benifit(0),symbol(0),charge_time(0){}
};


//����������
double cal_efficiency(double distance);
double double_rand(double start,double finish);
int int_rand(int start, int finish);
double dist(Point a,Point b);
double dis_route(vector<Point> vec);
void init();
void show(const vector<Point> &vec);
void improve_circle(vector<Point> &vec, double route_length);
void cal_benifit();
int choose_one_archor();
void choose_archor();
double charge_time(vector<Point> vec);
vector<vector<Point> > split_tsp(vector<Point> vec , double thread);
vector<vector<Point> > k_means(vector<Point> vec);
int find_min_core(vector<Point> core, Point spot);
int update_core(vector<Point> vec , Point& core_spot);
