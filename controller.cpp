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
    if(!(d>temporary.d)) return;
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

void Controller::optimize(const double *pos, double *dst)
{
    double delta=0.001;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.05;
    for(int i=0;i<2;i++)
    {
        Generator *pFuture;
        Generator *pAdd;
        Generator *mFuture;
        Generator *mAdd;

        double pPos[3]={pos[0],pos[1],pos[2]};
        double mPos[3]={pos[0],pos[1],pos[2]};

        pPos[i]=pos[i]+delta;
        mPos[i]=pos[i]-delta;
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

        gradient[i]=(cost(pfPath,paPath)-cost(mfPath,maPath))/(2.0*delta);
    }
    dst[0]=pos[0]-learning_rate*gradient[0];
    dst[1]=pos[1]-learning_rate*gradient[1];
}

double Controller::cost(std::vector<Generator::path> path, std::vector<Generator::path> aPath)
{
    double w1,w2;//weight
    double cost1,cost2;
    double mean_x,mean_y;
    double varianceX=0.0;
    double varianceY=0.0;

    w1=0.5;
    w2=10.0;

    //normalization
    std::vector<double> x(path.size());
    std::vector<double> y(path.size());

    for(int i=0;i<path.size();i++)
    {
        x[i]=path.at(i).px;
        y[i]=path.at(i).py;
    }
    for (int i=0;i<path.size()-1;i++)
    {
        double cross_product = x[i] * y[i+1] - x[i+1] * y[i];
        cost1+=labs(cross_product);
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
    auto start = std::chrono::high_resolution_clock::now();
    g->gen(Generator::prediction);
    //Generator *test=nullptr;

    double pos[3]={g->rPath.back().px,g->rPath.back().py,g->rPath.back().pq};
    pos[INDEX_Q]=g->addNoise(pos[INDEX_Q],RAD(3.0));
    pos[INDEX_X]=g->addNoise(pos[INDEX_X],0.05);
    pos[INDEX_Y]=g->addNoise(pos[INDEX_Y],0.05);
    pGen=new Generator(*g,pos);
    pGen->gen(Generator::stagnation);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function execution took " << duration.count() << " microseconds." << std::endl;
    int iLocalmin=-1;
    if(!checkGoal(pGen->getPath(),true))
    {
        if(pGen->isLocalmin())
        {
            iLocalmin=0;
        }
    }
    double opos[3]={rPos[0],rPos[1],rPos[2]};
    if(iLocalmin==0)
    {
        for(int i=0;i<100;i++)
        {
            opos[0]=g->addNoise(opos[0],0.05);
            opos[1]=g->addNoise(opos[1],0.05);
            double dst[2];
            optimize(opos,dst);
            opos[INDEX_X]=dst[INDEX_X];
            opos[INDEX_Y]=dst[INDEX_Y];
        }
        Generator* temp;
        Generator* atemp;
        temp=new Generator(*g,opos);
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
            else
            {
                iLocalmin=-1;
            }
        }
    }
    if(iLocalmin==-1)
    {
        double d=0.0;

        d=g->calcTemporaryGoal();

        // if(!(d<0.01))
        // {
        double tem[3];
        g->getTemporaryGoal(tem);
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
