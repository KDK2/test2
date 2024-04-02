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
    temporary_o({0,0,0,0,true}),
    rPos({5.0,9.0,0}),
    esum(0.0),
    kp(6.0),
    ki(0.006),
    minLoss(0.0),
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
    if(idle==state)
    {
        if(!(d>temporary.d)) return;
    }
    else if(optimized==state)
    {
        return;
    }
    temporary.x=x;
    temporary.y=y;
    temporary.theta=theta;
    temporary.arrived=false;
    temporary.d=d;
    esum=0.0;
}

void Controller::setOptimizedTemporaryGoal(double x, double y, double theta)
{
    temporary_o.arrived=false;
    temporary_o.x=x;
    temporary_o.y=y;
    temporary_o.theta=theta;
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
    double delta=0.1;

    std::vector<Generator::path> pfPath;
    std::vector<Generator::path> paPath;
    std::vector<Generator::path> mfPath;
    std::vector<Generator::path> maPath;

    double gradient[2]={0.0,0.0};
    double learning_rate=0.1;

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
    auto min_itr=std::min_element(l.begin(),l.end());
    loss=*min_itr;
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
    for (int i=0;i<path.size()-2;i++)
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
    if(-(w1*cost1+w2*cost2)<-0.3)
        std::cout<<px<<", "<<py<<" : "<<w1*cost1<<", "<<w2*cost2<<", "<<-(w1*cost1+w2*cost2)<<std::endl;
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
    if(isArrived())
    {
        return;
    }
    bool bDetect=false;
    detectLocalminimum(bDetect);
    if(localminimum==state)
    {
        if(bDetect==false)
            std::cout<<"optimized!"<<std::endl;
    }
    setState(bDetect);
    planing();
    moveGoal();
    updateGenerator();
}

void Controller::detectLocalminimum(bool& bLocalminimum)
{
    Generator* pRef=nullptr;
    Generator* pLocal=nullptr;
    Generator* pGen;
    double target[3];
    double lastPredict[3];
    double refPos[3];
    if(idle==state)
    {
        memcpy(refPos,rPos,sizeof(double)*3);
        pRef=g;
    }
    else if(localminimum==state)
    {
        double tg[3];
        getGoal(tg,false);//get temporary goal
        memcpy(refPos,tg,sizeof(double)*3);
        pLocal=new Generator(*g,refPos);
        pRef=pLocal;
    }
    else if(optimized==state)
    {
        memcpy(refPos,rPos,sizeof(double)*3);
        pRef=g;
    }
    getGoal(target,true);
    pRef->setPos(refPos);
    pRef->setGoal(target);
    pRef->gen(Generator::prediction);
    std::vector<Generator::path> temp;
    temp=pRef->getPath();
    lastPredict[0]=pRef->addNoise(temp.back().px,0.01);
    lastPredict[1]=pRef->addNoise(temp.back().py,0.01);
    lastPredict[2]=pRef->addNoise(temp.back().pq,RAD(0.2));
    pGen=new Generator(*pRef,lastPredict);
    pGen->gen(Generator::stagnation);
    pGen->getStagPos(stag_pos);
    std::vector<Generator::path> stag_path;
    stag_path=pGen->getPath();
    if(!checkGoal(pGen->getPath(),true))
    {
        if(pGen->isLocalmin())
        {
            bLocalminimum=true;
            if(localminimum==state)
            {
                s->addQuark(stag_pos[0],stag_pos[1]);
            }
        }
        else
        {
            bLocalminimum=false;
        }
    }
    else
    {
        bLocalminimum=false;
    }
    for(int i=0;i<stag_path.size();i++)
    {
        pRef->rPath.push_back(stag_path[i]);
    }
}

void Controller::setState(bool bLocalminimum)
{
    if(bLocalminimum)
    {
        if     (idle==state)         state=localminimum;
        else if(localminimum==state) state=localminimum;
        else if(optimized==state)    state=optimized;
    }
    else
    {
        if     (idle==state)         state=idle;
        else if(localminimum==state) state=optimized;
        else if(optimized==state)
        {
            state=idle;
        }
    }
}

void Controller::planing()
{
    //optimize when detect localminimum
    //this function run when state is idle or localminimum
    //set temporary goal
    Generator* ref=nullptr;
    double d=0.0;
    double tg[3];
    if(idle==state)
    {
        //calc temporary goal
        //set temporary goal
        ref=g;
        d=ref->calcTemporaryGoal();
        ref->getTemporaryGoal(tg);
        setTemporaryGoal(tg[0],tg[1],tg[2],d);
        return;
    }
    else if(localminimum==state)
    {
        o.clear();
        optimized_path.clear();
        int sgd_iter=200;
        double opos[3]={rPos[0],rPos[1],rPos[2]};
        std::vector<optimized_data>temp_o(sgd_iter);
        for(int i=0;i<sgd_iter;i++)
        {
            double temp=opos[0];
            double temp2=opos[1];
            opos[0]=g->addNoise(temp,0.2);
            opos[1]=g->addNoise(temp2,0.2);
            double dst[2];
            optimize(opos,dst,temp_o[i].cost1,temp_o[i].cost2,temp_o[i].loss);
            opos[0]=dst[0];
            opos[1]=dst[1];
            temp_o[i].x=opos[0];
            temp_o[i].y=opos[1];
            o.push_back(temp_o[i]);
        }
        if(o.size()>0)
        {
            std::vector<optimized_data> to;
            for(int i=0;i<o.size();i++)
            {
                if(o[i].loss<-0.25)
                    to.push_back(o[i]);
            }
            if(to.size()==0)
            {
                std::cout<<"no optimized data"<<std::endl;
                o.clear();
                return;
            }
            auto minIt = std::min_element(to.begin(), to.end(),
                                          [](const optimized_data& a, const optimized_data& b){return a.loss < b.loss;});
            int minIndex = std::distance(to.begin(), minIt);
            if(minLoss>to[minIndex].loss)
            {
                minLoss=to[minIndex].loss;
                o.clear();
                o.push_back(to[minIndex]);
            }
            else
            {
                o.clear();
                return;
            }
            opos[0]=to[minIndex].x;
            opos[1]=to[minIndex].y;
            ref=new Generator(*g,opos);
            ref->gen(Generator::prediction);
            optimized_path=ref->getPath();
            for(int i=0;i<optimized_path.size();i++)
            {
                g->rPath.push_back(optimized_path[i]);
            }
            double d =g->calcTemporaryGoal();
            g->getTemporaryGoal(tg);
            setTemporaryGoal(tg[0],tg[1],tg[2],d);
            std::cout<<"loss : "<<to[minIndex].loss<<std::endl;
            std::cout<<"goal: "<<tg[0]<<", "<<tg[1]<<std::endl;
            //setOptimizedTemporaryGoal(tg[0],tg[1],tg[2]);
            //temporary=temporary_o;
        }
        //run optimize
        //select best loss
        //calc temporary goal
        //set temporary goal
        return;
    }
    else if(optimized==state)
    {
        for(int i=0;i<optimized_path.size();i++)
        {
            g->rPath.push_back(optimized_path[i]);
        }
        double d =g->calcTemporaryGoal();
        g->getTemporaryGoal(tg);
        setTemporaryGoal(tg[0],tg[1],tg[2],d);
    }
}

void Controller::moveGoal()
{
    //this function run when state is idle or optimized
    if(localminimum==state)
        return;

    double tg[3];
    double v_ref,q_ref,v,w;
    double ref[2];
    getGoal(tg,false);
    g->setGoal(tg);
    g->gen(Generator::reference);
    g->getRef(v_ref,q_ref);
    ref[0]=v_ref;
    ref[1]=q_ref;
    velocity(ref,v,w);
    a->update(rPos,rPos,v,w);
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
