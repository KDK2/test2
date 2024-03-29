#include "controller.h"
#include "math.h"
#define INDEX_REF_V 0
#define INDEX_REF_Q 1

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2
static std::ofstream file("data.csv");
static int iter=0;
Controller::Controller():
    s(nullptr),
    a(nullptr),
    g(nullptr),
    temporary({0,0,0,0,true}),
    rPos({0,0,0}),
    esum(0.0),
    kp(6.0),
    ki(0.006),
    state(idle)
{
}

Controller::~Controller()
{
    file.close();
    delete g;
    delete a;
    delete s;
}

void Controller::setSensor(Sensor *sensor)
{
    if(s!=nullptr)
    {
        delete s;
    }
    s=sensor;
}

void Controller::setActuator(Actuator *actuator)
{
    if(a!=nullptr)
    {
        delete a;
    }
    a=actuator;
}

void Controller::setGenerator(Generator *generator)
{
    if(g!=nullptr)
    {
        delete g;
    }
    g=generator;
}

void Controller::setTemporaryGoal(double x, double y, double theta, double d)
{
    //if(!(d>temporary.d)) return;
    temporary.x=x;
    temporary.y=y;
    temporary.theta=theta;
    temporary.arrived=false;
    temporary.d=d;
    //s->vq.clear();
    esum=0.0;
}

void Controller::addGoal(double x, double y, double theta)
{
    goal g;
    g.x=x;
    g.y=y;
    g.theta=theta;
    g.arrived=false;
    goals.push_back(g);
}

void Controller::checkMaxVelocity(double vel, double vel_max, double &dst)
{
    if(abs(vel)<=vel_max)
    {
        dst=vel;
    }
    else
    {
        dst=vel_max*vel/abs(vel);
    }
}
#include <algorithm>
void Controller::optimize(const double *pos, double *dst, double* cst1, double* cst2, double& loss)
{
    double delta=0.001;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.01;

    std::vector<double> cost1;
    std::vector<double> cost2;
    std::vector<double> l;
    for(int i=0;i<2;i++)
    {
        Generator *pFuture;
        Generator *pAdd;
        Generator *mFuture;
        Generator *mAdd;

        double pPos[3]={pos[0],pos[1],pos[2]};
        double mPos[3]={pos[0],pos[1],pos[2]};

        pPos[i]+=delta;
        mPos[i]-=delta;
        pFuture=new Generator(*g,pPos);
        mFuture=new Generator(*g,mPos);
        pFuture->gen(Generator::prediction);
        mFuture->gen(Generator::prediction);

        pfPath=pFuture->getPath();
        mfPath=mFuture->getPath();

        double paPos[3]={pfPath.back().px,pfPath.back().py,pfPath.back().pq};
        double maPos[3]={mfPath.back().px,mfPath.back().py,mfPath.back().pq};

        pAdd=new Generator(*g,paPos);
        mAdd=new Generator(*g,maPos);

        pAdd->gen(Generator::stagnation);
        mAdd->gen(Generator::stagnation);

        paPath=pAdd->getPath();
        maPath=mAdd->getPath();
        double pc[2];//plus delta cost1,2
        double mc[2];//minus delta cost1,2
        double ploss;
        double mloss;
        double pl=cost(pfPath,paPath,pc,ploss);
        double ml=cost(mfPath,maPath,mc,mloss);
        gradient[i]=(pl-ml)/(2.0*delta);
        cost1.push_back(pc[0]);
        cost1.push_back(mc[0]);
        cost2.push_back(pc[1]);
        cost2.push_back(mc[1]);
        l.push_back(ploss);
        l.push_back(mloss);
    }
    for(int i=0;i<cost1.size();i++)
    {
        cst1[i]=cost1[i];
        cst2[i]=cost2[i];
    }
    auto max_itr=std::max_element(l.begin(),l.end());
    loss=*max_itr;
    dst[0]=pos[0]-learning_rate*gradient[0];
    dst[1]=pos[1]-learning_rate*gradient[1];
}

double Controller::cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath, double* cst, double& loss)
{
    double w1,w2;//weight
    double cost1,cost2;
    double mean_x,mean_y;
    double varianceX=0.0;
    double varianceY=0.0;

    w1=0.8;
    w2=10.0;

    //normalization
    std::vector<double> x(path.size());
    std::vector<double> y(path.size());
    std::vector<double> diff_x(path.size()-1);
    std::vector<double> diff_y(path.size()-1);
    std::vector<double> distance(path.size()-1);
    std::vector<double> normalized_distance(path.size()-1);
    std::vector<double> normalized_x(path.size()-1);
    std::vector<double> normalized_y(path.size()-1);
    double total_distance=0.0;
    for(int i=0;i<path.size();i++)
    {
        x[i]=path.at(i).px;
        y[i]=path.at(i).py;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        diff_x[i]=x[i+1]-x[i];
        diff_y[i]=y[i+1]-y[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        distance[i]=sqrt(pow(x[i+1]-x[i],2)+pow(y[i+1]-y[i],2));
    }
    for(int i=0;i<path.size()-1;i++)
    {
        total_distance+=distance[i];
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_distance[i]=distance[i]/total_distance;
    }
    for(int i=0;i<path.size()-1;i++)
    {
        normalized_x[i]=normalized_distance[i]*diff_x[i]/distance[i];
        normalized_y[i]=normalized_distance[i]*diff_y[i]/distance[i];
    }
    double cum_sum_x=0.0;
    double cum_sum_y=0.0;
    for(int i=0;i<path.size()-1;i++)
    {
        cum_sum_x+=normalized_x[i];
        cum_sum_y+=normalized_y[i];
        x[i]=cum_sum_x;
        y[i]=cum_sum_y;
    }
    for (int i=0;i<path.size()-1;i++)
    {
        double cross_product =x[i]*y[i+1]-x[i+1]*y[i];
        cost1+=abs(cross_product);
    }

    mean_x=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.px;});
    mean_x/=aPath.size();
    mean_y=std::accumulate(aPath.begin(),aPath.end(),0.0,[](double sum, Generator::path p){return sum+p.py;});
    mean_y/=aPath.size();

    for(int i=0;i<aPath.size();i++)
    {
        varianceX+=pow(aPath.at(i).px-mean_x,2);
        varianceY+=pow(aPath.at(i).py-mean_y,2);
    }
    varianceX/=aPath.size();
    varianceY/=aPath.size();
    cost2=varianceX+varianceY;
    cst[0]=w1*cost1;
    cst[1]=w2*cost2;
    double px,py;
    px=path.front().px;
    py=path.front().py;
    if(w2*cost2>0.2)
    {
        std::cout<<px<<", "<<py<<" : "<<cost1<<", "<<cost2<<", "<<-(w1*cost1+w2*cost2)<<std::endl;
    }
    loss=-(w1*cost1+w2*cost2);
    return -(w1*cost1+w2*cost2);
}

void Controller::checkGoal()
{
    double goal[3];
    getGoal(goal,false);

    double dx=goal[INDEX_X]-rPos[INDEX_X];
    double dy=goal[INDEX_Y]-rPos[INDEX_Y];
    double d=sqrt(dx*dx+dy*dy);
    double tolorance=g->ip.m_param.eparam.tolorance;
    if(!temporary.arrived)
    {
        if(d<tolorance)
        {
            temporary.arrived=true;
            temporary.d=0.0;
            //s->vq.clear();
        }
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(goals[i].arrived) continue;
            if(d<tolorance) goals[i].arrived=true;
        }
    }
}

bool Controller::isArrived()
{
    bool ret=true;
    for(int i=0;i<goals.size();i++)
    {
        ret&=goals[i].arrived;
    }
    return ret;
}

bool Controller::checkGoal(std::vector<Generator::path> path,bool bGlobal)
{
    double goal[3];
    getGoal(goal,bGlobal);
    for(int i=0;i<path.size();i++)
    {
        double dx=path[i].px-goal[INDEX_X];
        double dy=path[i].py-goal[INDEX_Y];
        double d=sqrt(dx*dx+dy*dy);
        if(d<g->ip.m_param.eparam.tolorance) return true;
    }
    return false;
}

void Controller::velocity(double *src, double &v, double &w)
{
    double v_max=g->ip.m_param.vparam.v_max;
    double w_max=g->ip.m_param.vparam.w_max;
    double v_ref=src[INDEX_REF_V];
    double q_ref=src[INDEX_REF_Q];
    double e;
    g->normalizeAngle(q_ref-rPos[INDEX_Q],e);

    esum+=e;
    checkMaxVelocity(v_ref,v_max,v);
    checkMaxVelocity(kp*e+ki*esum,w_max,w);
}
#define RAD(x) ((x)*M_PI/180.0)
#include <chrono>
void Controller::control()
{
    iter++;
    if(iter>250)
    {
        int de=0;
    }
    if(isArrived())
    {
        return;
    }
    double lam=g->ip.p_param.lparam.lam;
    double lam_stagnation=g->ip.p_param.lparam.lam_stagnation;
    double delta=g->ip.p_param.lparam.delta;
    int    iter_max=(lam_stagnation)/delta;

    Generator* pGen;//stagnation genarator
    double goal[3];
    getGoal(goal,true);
    g->setPos(rPos);
    g->setGoal(goal);
    // auto start = std::chrono::high_resolution_clock::now();
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Function execution took " << duration.count() << " microseconds." << std::endl;
    g->gen(Generator::prediction);
    //Generator *test=nullptr;

    double pos[3]={g->rPath.back().px,g->rPath.back().py,g->rPath.back().pq};
    pos[INDEX_Q]=g->addNoise(pos[INDEX_Q],RAD(3.0));
    pos[INDEX_X]=g->addNoise(pos[INDEX_X],0.05);
    pos[INDEX_Y]=g->addNoise(pos[INDEX_Y],0.05);
    pGen=new Generator(*g,pos);
    pGen->gen(Generator::stagnation);
    int iLocalmin=-1;
    if(!checkGoal(pGen->getPath(),true))
    {
        if(pGen->isLocalmin())
        {
            iLocalmin=0;
        }
    }
    int sgd_iter=200;
    double opos[3]={rPos[0],rPos[1],rPos[2]};
    double oppos[3]={rPos[0],rPos[1],rPos[2]};
    std::vector<optimized> temp_o(sgd_iter);
    Generator* temp=nullptr;
    bool bfirst=true;
    if(iLocalmin==0)
    {
        o.clear();
        for(int i=0;i<sgd_iter;i++)
        {
            opos[0]=g->addNoise(opos[0],0.01);
            opos[1]=g->addNoise(opos[1],0.01);
            double dst[2];
            optimize(opos,dst,temp_o[i].cost1,temp_o[i].cost2,temp_o[i].loss);
            opos[INDEX_X]=dst[INDEX_X];
            opos[INDEX_Y]=dst[INDEX_Y];
            temp_o[i].x=opos[INDEX_X];
            temp_o[i].y=opos[INDEX_Y];
            if(temp_o[i].cost2[0]>0.2)
            {
                if(temp_o[i].loss<-2.0)
                {
                    o.push_back(temp_o[i]);
                }
            }
        }
        if(o.size()>0)
        {
            //find min index
            int min_index=0;
            double min_loss=o[0].loss;
            for(int i=1;i<o.size();i++)
            {
                if(min_loss>o[i].loss)
                {
                    min_loss=o[i].loss;
                    min_index=i;
                }
            }
            oppos[INDEX_X]=o[min_index].x;
            oppos[INDEX_Y]=o[min_index].y;
            std::cout<<"min_loss : "<<min_loss<<std::endl;
            Generator* atemp;
            temp=new Generator(*g,oppos);
            temp->gen(Generator::prediction);
            double apos[3]={temp->getPath().back().px,temp->getPath().back().py,temp->getPath().back().pq};
            atemp=new Generator(*g,apos);
            atemp->gen(Generator::stagnation);
            if(!checkGoal(atemp->getPath(),true))
            {
                if(atemp->isLocalmin())
                {
                    iLocalmin=0;
                }
            }
            else
            {
                iLocalmin=-1;
            }
        }
    }
    if(iLocalmin==-1)
    {
        double d=0.0;
        Generator* temp_g=nullptr;
        if(temp!=nullptr)
        {
            temp_g=temp;
        }
        else
        {
            temp_g=g;
        }
        d=temp_g->calcTemporaryGoal();

        // if(!(d<0.01))
        // {
        double tem[3];
        temp_g->getTemporaryGoal(tem);
        setTemporaryGoal(tem[INDEX_X],tem[INDEX_Y],tem[INDEX_Q],d);//temporary goal의 생성 기준이 필요하다...
        state=idle;
        updateGenerator();
        getGoal(goal,false);
        g->setGoal(goal);
        double v_ref,q_ref,v,w;
        g->gen(Generator::reference);
        g->getRef(v_ref,q_ref);

        double ref[2]={v_ref,q_ref};
        velocity(ref,v,w);
        a->update(rPos,rPos,v,w);
    }
    else
    {
        double qpos[2];

        if(state==idle)
        {
            state=localminimum;
        }
        pGen->getStagPos(qpos);
        s->addQuark(qpos[INDEX_X],qpos[INDEX_Y]);

        //temporary골의 선택
        //temporary T 점에서 robot - goal 과의 수직하는 거리가 새로운 temporary goal T_n이 계산하는 robot - goal거리 d_n
        //보다 작으면 새로운 temporary 골이라고 정한다.
    }
    for(int i=0;i<pGen->rPath.size();i++)
    {
        g->rPath.push_back(pGen->rPath[i]);
    }
    updateGenerator();
    // if(!file.is_open())
    // {
    //     std::cerr<<"fail"<<std::endl;
    //     return;
    // }
    // file << pGen->getVariance()<<std::endl;
}

void Controller::getPos(double *dst)
{
    dst[INDEX_X]=rPos[INDEX_X];
    dst[INDEX_Y]=rPos[INDEX_Y];
    dst[INDEX_Q]=rPos[INDEX_Q];
}

void Controller::getGoal(double *dst,bool bGlobal)
{
    if(bGlobal)
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
    if(!temporary.arrived)
    {
        dst[INDEX_X]=temporary.x;
        dst[INDEX_Y]=temporary.y;
        dst[INDEX_Q]=temporary.theta;
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
}

void Controller::updateGenerator()
{
    g->updateSensor(*s);
    checkGoal();
}
