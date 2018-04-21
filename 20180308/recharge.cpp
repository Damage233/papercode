#include "recharge.h"
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
