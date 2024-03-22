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
void Controller::control()
{
    iter++;
    if(iter>300)
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
    updateGenerator();
    if(!file.is_open())
    {
        std::cerr<<"fail"<<std::endl;
        return;
    }
    file << pGen->getVariance()<<std::endl;
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
