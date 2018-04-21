#include <iostream>
#include "recharge.h"
using namespace std;

//ȫ�ֱ�����
double d[N_NODE][N_NODE];                             //ȫ�ֱ���������ڵ�֮�����̾���
vector<Point> p;                            //�������нڵ�
vector<Point> p_archor;                     //��������ê��
int sum = 0;                                //��ǰ·����̾���
//int w = 100;                                //�޶���Ⱥֻ�ܻ�100������
vector<vector<Point> > group;               //��Ⱥ��Ҳ����Ⱦɫ���б�

int main()
{
    init();             //��ʼ���ڵ�

    double route_P = dis_route(p);   //·������
    cout<<"��ʼ·�����ȣ�"<<route_P<<endl;

    cal_benifit();      //����ÿ���ڵ�����棬Ϊ̰��ѡ��ê����׼��
    choose_archor();    //ê��ѡ�񣬸���ê�㼯�� p_archor

    double befor_improve_P = dis_route(p_archor);
    cout << "�Ż�֮ǰê���·����";
    show(p_archor);

    cout << "����Ȧ���Ż����·���� ";
    improve_circle(p_archor,befor_improve_P);
    show(p_archor);

    cout<<"----------------------------------------------------------------"<<endl;
    vector<vector<Point> > k_means_sencar;
    k_means_sencar = k_means(p_archor);
    double singe_TT = 0 ;      //��¼�����ʱ��
    for(int i = 0; i< k_means_sencar.size(); ++i)
    {
        cout<<"��"<<i+1<<"��:"<<endl;
        double chargeTT = charge_time(k_means_sencar[i]);
        double moveTT = dis_route(k_means_sencar[i])/SPEED;
        cout<<"���ʱ�����ģ� "<<chargeTT<<endl;
        cout<<"�ƶ�ʱ�����ģ� "<<moveTT<<endl;
        show(k_means_sencar[i]);
        if(singe_TT < chargeTT + moveTT )
        {
            singe_TT = chargeTT + moveTT;
        }
    }
    cout<<"�����ʱ��:"<<singe_TT<<" "<<singe_TT / 60<<endl;
    cout<<"----------------------------------------------------------------"<<endl;


    double charge_T = charge_time(p_archor);
    cout<<"��TSP�����ĵĳ��ʱ��Ϊ��"<<charge_T<<"s"<<endl;

    double threshold_t = charge_T / M_SENCAR;
    cout<<threshold_t<<endl;

    vector<vector<Point> > sencar_tsp = split_tsp(p_archor,threshold_t);
    double singe_T = 0 ;      //��¼�����ʱ��
    for(int i = 0; i< sencar_tsp.size(); ++i)
    {
        cout<<"��"<<i+1<<"��sencar���ߵ�·��:"<<endl;
        double chargeT = charge_time(sencar_tsp[i]);
        double moveT = dis_route(sencar_tsp[i])/SPEED;
        cout<<"���ʱ�����ģ� "<<chargeT<<endl;
        cout<<"�ƶ�ʱ�����ģ� "<<moveT<<endl;
        show(sencar_tsp[i]);
        if(singe_T < chargeT + moveT )
        {
            singe_T = chargeT + moveT;
        }
    }
    cout<<"�����ʱ��:"<<singe_T<<" "<<singe_T / 60<<endl;
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
    //��ʼ���ڵ���Ϣ
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
    //��ʼ���������
    for(int i = 0; i < N_NODE; ++i)
    {
        for(int j=0; j < N_NODE; ++j)
        {
            d[i][j] =dist(p[i],p[j]);
        }
    }
    //·������
    //sum = dis_route(p);

}
void show(const vector<Point> &vec)
{
    cout<<"·�����ȣ�"<<dis_route(vec)<<endl;
//    cout<<"·��:"<<endl;
//    for(int i = 0; i < vec.size() ; ++i)
//    {
//        cout<<vec[i].number<<" "<<vec[i].group_number<<" "<<vec[i].coordinate_x<<" "<<vec[i].coordinate_y<<" "<<vec[i].charge_time<<endl;;
//    }
    cout<<endl;

}
void improve_circle(vector<Point> &vec, double route_length) { //����Ȧ���õ���ʼ����
    vector<Point> cur = vec;
    for (int t = 0; t < ROUND ; t++) {     //�ظ�50��
        for (int i = 0; i < vec.size(); i++) { //�������˳��
            int j = rand() % vec.size();
            swap(cur[i], cur[j]);
        }
        int flag = 1;
        while (flag) {
            flag = 0;
            //����ѡȡuv�Ӵ������Ե���uv�Ӵ���˳�����Ƿ���ţ������������
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
        int si = 0; // ���ڼ���
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
        p[arch].symbol = 1;         //���ʹ� �����ٴη���
        for(int i = 0; i < N_NODE ; ++i)
        {
            if(d[arch][i] <= MAX_HOP_DIS)
            {
                p[i].efficiency = cal_efficiency(d[arch][i]);
                p[i].symbol = 1 ;
                p[i].group_number = arch ; //�������
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
        double power;       //sencar����
        if(num_node >= SIMUL_CHRAGE_NODE)
            power = P_SENCAR ;
        else
            power = P_SENCAR * num_node / SIMUL_CHRAGE_NODE;

        p[arch].charge_time = charge / power;

        p_archor.push_back(p[arch]);    //��ѡ���ê����뵽 p_archor��

        arch = choose_one_archor();
        k++;
    }
    //cout<<endl;
    cout<<"ê�������"<<k<<endl;

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
    vector<Point> core;     //����
    vector<vector<Point> > deployment;
    for(int i = 0; i< M_SENCAR; ++i){
        vector<Point> temp;
        deployment.push_back(temp);
    }
//��ʼ�����ģ�k = M_SENCAR
    for(int i = 0; i < M_SENCAR; ++i){
        Point spot;
        spot.coordinate_x = vec[vec.size()*i/M_SENCAR].coordinate_x;
        spot.coordinate_y = vec[vec.size()*i/M_SENCAR].coordinate_y;
        core.push_back(spot);
    }
//����ڵ㲢���´���
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



















