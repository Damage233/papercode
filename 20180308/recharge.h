//头文件一般包含类的定义、extern变量的声明和函数声明，const对象
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

//const常量：
const double P_SENCAR = 10;                             //Sencar的最大输出功率，单位J/S
const int SIMUL_CHRAGE_NODE = 4;                        //当节点大于它时，Sencar达到最大功率
const double BATTERY_ENERGY = 1000;                     //传感器满电量,单位J(焦耳）
const double MOBILE_CONSUMPTION = 30;                   //Sencar的移动消耗J/s
const double SPEED = 5;                                 //Sencar的移动速率单位m/s
const int N_NODE = 100;                                 //传感器网络中节点的数量
const int M_SENCAR = 3;                                 //sencars数量
const double LENGTH = 30;                               //传感器网络的长度
const double WIDTH = 30;                                //传感器网络的宽度
const double INF = 32767;
const double MAX_HOP_DIS = 1.5;                         //中继充电单跳最大距离
const double MAX_HOP = 1;                               //中继充电总距离限制
const int ROUND = 100 ;


//结构体定义：
class Point
{
public:
    int number;                 //节点序号，主键 不同节点的唯一区别
    int group_number;           //中继充电组的组号
//    int father_node;            //后继节点的序号
//    int sum_children;           //子节点的个数，用于计算节点的能耗
    bool anchor;                //该节点是否为锚节点

    double coordinate_x;        //x坐标
    double coordinate_y;        //y坐标
    double current_enery;       //传感器的当前能量
    double efficiency;          //充电效率
    double benifit;             //以该点作为锚点的收益

    int symbol;                 //一个标记，锚点选择的时候是否被遍历过
    double charge_time;

public:
    Point():number(0),group_number(0),anchor(true),coordinate_x(0),coordinate_y(0),
    current_enery(BATTERY_ENERGY),efficiency(1),benifit(0),symbol(0),charge_time(0){}
};


//函数声明：
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
