#include <iostream>
#include "recharge.h"
using namespace std;

//全局变量：
double d[N_NODE][N_NODE];                             //全局变量，保存节点之间的最短距离
vector<Point> p;                            //保存所有节点
vector<Point> p_archor;                     //保存所有锚点
int sum = 0;                                //当前路径最短距离
//int w = 100;                                //限定种群只能活100个个体
vector<vector<Point> > group;               //种群，也就是染色体列表

int main()
{
    init();             //初始化节点

    double route_P = dis_route(p);   //路径长度
    cout<<"初始路径长度："<<route_P<<endl;

    cal_benifit();      //计算每个节点的收益，为贪心选择锚点做准备
    choose_archor();    //锚点选择，更新锚点集合 p_archor

    double befor_improve_P = dis_route(p_archor);
    cout << "优化之前锚点的路径：";
    show(p_archor);

    cout << "改良圈法优化后的路径： ";
    improve_circle(p_archor,befor_improve_P);
    show(p_archor);

    cout<<"----------------------------------------------------------------"<<endl;
    vector<vector<Point> > k_means_sencar;
    k_means_sencar = k_means(p_archor);
    double singe_TT = 0 ;      //记录充电总时间
    for(int i = 0; i< k_means_sencar.size(); ++i)
    {
        cout<<"第"<<i+1<<"类:"<<endl;
        double chargeTT = charge_time(k_means_sencar[i]);
        double moveTT = dis_route(k_means_sencar[i])/SPEED;
        cout<<"充电时间消耗： "<<chargeTT<<endl;
        cout<<"移动时间消耗： "<<moveTT<<endl;
        show(k_means_sencar[i]);
        if(singe_TT < chargeTT + moveTT )
        {
            singe_TT = chargeTT + moveTT;
        }
    }
    cout<<"充电总时间:"<<singe_TT<<" "<<singe_TT / 60<<endl;
    cout<<"----------------------------------------------------------------"<<endl;


    double charge_T = charge_time(p_archor);
    cout<<"单TSP所消耗的充电时间为："<<charge_T<<"s"<<endl;

    double threshold_t = charge_T / M_SENCAR;
    cout<<threshold_t<<endl;

    vector<vector<Point> > sencar_tsp = split_tsp(p_archor,threshold_t);
    double singe_T = 0 ;      //记录充电总时间
    for(int i = 0; i< sencar_tsp.size(); ++i)
    {
        cout<<"第"<<i+1<<"辆sencar行走的路径:"<<endl;
        double chargeT = charge_time(sencar_tsp[i]);
        double moveT = dis_route(sencar_tsp[i])/SPEED;
        cout<<"充电时间消耗： "<<chargeT<<endl;
        cout<<"移动时间消耗： "<<moveT<<endl;
        show(sencar_tsp[i]);
        if(singe_T < chargeT + moveT )
        {
            singe_T = chargeT + moveT;
        }
    }
    cout<<"充电总时间:"<<singe_T<<" "<<singe_T / 60<<endl;
}

double cal_efficiency(double distance)
{
    return -0.4057*pow(distance,2) + 0.2506*distance + 0.8820;
}

double double_rand(double start,double finish)
{
    int range = static_cast<int>(finish - start);
    return start + static_cast<double>(rand()%(range*100)) / 100;
}
int int_rand(int start, int finish)
{
    return start + rand()%(finish - start + 1);
}
double dist(Point a,Point b)
{
    return sqrt(pow((a.coordinate_x - b.coordinate_x),2) + pow((a.coordinate_y - b.coordinate_y),2));
}
double dis_route(vector<Point> vec)
{
    double route = 0 ;
    for(int i=1 ; i < vec.size(); ++i)
    {
        route += d[vec[i].number][vec[i-1].number];
    }
    route += d[vec[0].number][vec[vec.size()-1].number];
    return route;
}
void init()
{
    p.clear();
    //初始化节点信息
    for(int i = 0;i < N_NODE ;++i)
    {
        Point t;
        t.number = i;
        t.group_number = i;
        t.coordinate_x = double_rand(0,WIDTH);
        t.coordinate_y = double_rand(0,LENGTH);
        t.current_enery = int_rand(0,BATTERY_ENERGY);
        p.push_back(t);
    }
    //初始化距离矩阵
    for(int i = 0; i < N_NODE; ++i)
    {
        for(int j=0; j < N_NODE; ++j)
        {
            d[i][j] =dist(p[i],p[j]);
        }
    }
    //路径长度
    //sum = dis_route(p);

}
void show(const vector<Point> &vec)
{
    cout<<"路径长度："<<dis_route(vec)<<endl;
//    cout<<"路径:"<<endl;
//    for(int i = 0; i < vec.size() ; ++i)
//    {
//        cout<<vec[i].number<<" "<<vec[i].group_number<<" "<<vec[i].coordinate_x<<" "<<vec[i].coordinate_y<<" "<<vec[i].charge_time<<endl;;
//    }
    cout<<endl;

}
void improve_circle(vector<Point> &vec, double route_length) { //改良圈法得到初始序列
    vector<Point> cur = vec;
    for (int t = 0; t < ROUND ; t++) {     //重复50次
        for (int i = 0; i < vec.size(); i++) { //构造随机顺序
            int j = rand() % vec.size();
            swap(cur[i], cur[j]);
        }
        int flag = 1;
        while (flag) {
            flag = 0;
            //不断选取uv子串，尝试倒置uv子串的顺序后解是否更优，如果更优则变更
            for (int u = 1; u < vec.size() - 2; u++) {
                for (int v = u + 1; v < vec.size() - 1; v++) {
                    if (d[cur[u].number][cur[v + 1].number] + d[cur[u - 1].number][cur[v].number] <
                        d[cur[u].number][cur[u - 1].number] + d[cur[v].number][cur[v + 1].number]) {
                        for (int k = u; k <= (u + v) / 2; k++) {
                            swap(cur[k], cur[v - (k - u)]);
                            flag = 1;
                        }
                    }
                }
            }
        }
        group.push_back(cur);
        double cur_sum = dis_route(cur);
        if (cur_sum < route_length) {
            route_length = cur_sum;
            vec = cur;
        }
    }
}
void cal_benifit()
{
    for(int i = 0 ; i < N_NODE ;++i)
    {
        p[i].benifit = 0;
        int si = 0; // 用于计数
        for(int j = 0; j < N_NODE ; ++j){
            if(d[i][j] <= MAX_HOP_DIS){
                double nij = cal_efficiency(d[i][j]);
                p[i].benifit += (1-nij)/nij*(BATTERY_ENERGY - p[j].current_enery);
                ++si;
            }
        }
        p[i].benifit = p[i].benifit / si ;
    }
}
int choose_one_archor()
{
    int index = -1;
    double min_benifite = INF;
    for(int i = 0; i < N_NODE ;++i)
    {
        if(p[i].symbol == 0 && p[i].benifit < min_benifite)
        {
            index = i;
            min_benifite =p[i].benifit;
        }
    }
    return index;
}
void choose_archor()
{
    int arch = choose_one_archor();
    p_archor.clear();
    int k = 0;
    while(arch != -1)
    {
//        cout<<arch<<endl;
        p[arch].symbol = 1;         //访问过 ，不再次访问
        for(int i = 0; i < N_NODE ; ++i)
        {
            if(d[arch][i] <= MAX_HOP_DIS)
            {
                p[i].efficiency = cal_efficiency(d[arch][i]);
                p[i].symbol = 1 ;
                p[i].group_number = arch ; //设置组号
                if(arch != i)
                {
                    p[i].anchor = false;
                }
            }
        }

        double charge = 0;
        int num_node = 0;
        for(int i = 0; i < p.size(); ++i)
        {
            if(arch == p[i].number)
            {
                charge += (BATTERY_ENERGY - p[i].current_enery) /p[i].efficiency;
                ++num_node;
            }

        }
        double power;       //sencar功率
        if(num_node >= SIMUL_CHRAGE_NODE)
            power = P_SENCAR ;
        else
            power = P_SENCAR * num_node / SIMUL_CHRAGE_NODE;

        p[arch].charge_time = charge / power;

        p_archor.push_back(p[arch]);    //把选择的锚点插入到 p_archor中

        arch = choose_one_archor();
        k++;
    }
    //cout<<endl;
    cout<<"锚点个数："<<k<<endl;

}
double charge_time(vector<Point> vec)
{
    double charge = 0;
    for(vector<Point>::iterator iter = vec.begin() ; iter != vec.end(); ++iter)
    {
        charge += (*iter).charge_time;

    }
    return charge;
}

vector<vector<Point> > split_tsp(vector<Point> vec , double thread)
{
    vector<vector<Point> > deployment;
    vector<Point> box;
    double cur_time = 0 ;
    for(vector<Point>::iterator iter = vec.begin(); iter != vec.end(); ++iter)
    {
        box.push_back(*iter);
        cur_time += (*iter).charge_time;
        if(cur_time >= thread)
        {
            deployment.push_back(box);
            box.clear();
            cur_time = 0;
        }
    }
    if(box.begin() != box.end())
    {
        deployment.push_back(box);
    }
    return deployment;
}

vector<vector<Point> > k_means(vector<Point> vec){
    vector<Point> core;     //簇心
    vector<vector<Point> > deployment;
    for(int i = 0; i< M_SENCAR; ++i){
        vector<Point> temp;
        deployment.push_back(temp);
    }
//初始化簇心，k = M_SENCAR
    for(int i = 0; i < M_SENCAR; ++i){
        Point spot;
        spot.coordinate_x = vec[vec.size()*i/M_SENCAR].coordinate_x;
        spot.coordinate_y = vec[vec.size()*i/M_SENCAR].coordinate_y;
        core.push_back(spot);
    }
//分配节点并更新簇心
    int sum = 1;
    int count_k_means = 0;
    while(count_k_means < 100 && sum){
        for(int i = 0; i < deployment.size(); ++i){
            deployment[i].clear();
        }
        for(int i = 0; i < vec.size(); ++i){
            int j = find_min_core(core,vec[i]);
            deployment[j].push_back(vec[i]);
        }
        sum = 0;
        for(int i = 0; i < deployment.size(); ++i){
            int k = update_core(deployment[i],core[i]);
            sum += k;
        }
        ++count_k_means;
    }
    return deployment;

}
int find_min_core(vector<Point> core, Point spot){
    int index = 0;
    double min_dist = dist(spot,core[0]);
    for(int i = 1; i < core.size(); ++i){
        if(min_dist > dist(spot,core[i])){
            min_dist = dist(spot,core[i]);
            index = i;
        }
    }
    return index;
}
int update_core(vector<Point> vec , Point& core_spot){
    Point temp;
    for(int i = 0; i < vec.size(); ++i){
        temp.coordinate_x += vec[i].coordinate_x;
        temp.coordinate_y += vec[i].coordinate_y;
    }
    temp.coordinate_x = temp.coordinate_x / vec.size();
    temp.coordinate_y = temp.coordinate_y / vec.size();
    if(core_spot.coordinate_x == temp.coordinate_x&&core_spot.coordinate_y == temp.coordinate_y){
        return 0;
    }else{
        core_spot.coordinate_x = temp.coordinate_x;
        core_spot.coordinate_y = temp.coordinate_y;
        return 1;
    }
}



















