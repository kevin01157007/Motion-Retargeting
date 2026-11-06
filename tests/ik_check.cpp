#include "motion.hpp"
#include <cstdio>
#include <cmath>
#include <fstream>

static bool finiteQ(const Quat& q){return std::isfinite(q.w)&&std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z);}
static double dist(Vec3 a, Vec3 b){return length(a-b);}

static Skeleton makeLeg(){
    Skeleton s;
    s.joints.push_back({"Hips",-1,{0,1.0,0}});
    s.joints.push_back({"UpLeg",0,{0.1,0,0}});
    s.joints.push_back({"Leg",1,{0,-0.45,0}});
    s.joints.push_back({"Foot",2,{0,-0.42,0}});
    return s;
}
static Pose identityPose(const Skeleton& s){
    Pose p; p.localRotation.assign(s.joints.size(), Quat{1,0,0,0}); return p;
}
static Vec3 footPos(const Skeleton& s,const Pose& p,int foot){
    return forwardKinematics(s,p).position[foot];
}

int main(){
    Skeleton sk = makeLeg(); const int FOOT=3, HIP=1;
    int fails=0;
    std::ofstream csv("ik_convergence.csv"); csv<<"case,iter,error_m\n";

    // ---- (A) 可達目標 ----
    Pose start = identityPose(sk);
    Pose ref = start;
    ref.localRotation[1] = axisAngle({1,0,0}, 0.5);    // 大腿 ~29°
    ref.localRotation[2] = axisAngle({1,0,0}, -0.7);   // 膝蓋 ~40°
    Vec3 target = footPos(sk, ref, FOOT);              // 保證可達

    double before = dist(footPos(sk,start,FOOT), target);
    Pose out = start; solveCCD(sk,out,FOOT,target,30);
    double after = dist(footPos(sk,out,FOOT), target);
    std::printf("[reachable] before=%.4f m  after=%.5f m\n",before,after);
    if(!(after<0.01)) {std::puts("FAIL: not within 1 cm"); ++fails;}

    // 逐次迭代誤差(給圖用)
    for(int it=0; it<=30; ++it){
        Pose p = start; solveCCD(sk,p,FOOT,target,it);
        csv<<"reachable,"<<it<<","<<dist(footPos(sk,p,FOOT),target)<<"\n";
    }

    // ---- (B) 不可達目標 ----
    Vec3 far = footPos(sk,start,FOOT) + Vec3{0,0,3.0};
    Pose pf = start; solveCCD(sk,pf,FOOT,far,30);
    WorldPose w = forwardKinematics(sk,pf);
    bool ok = true;
    for(auto& q: pf.localRotation) ok = ok && finiteQ(q);
    for(auto& v: w.position) ok = ok && std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);
    double d0 = dist(footPos(sk,start,FOOT), far), d1 = dist(w.position[FOOT], far);
    std::printf("[unreachable] finite=%d  before=%.4f after=%.4f\n",ok,d0,d1);
    if(!ok || !(d1<=d0+1e-9)) {std::puts("FAIL: unreachable case"); ++fails;}

    // ---- (C) 魯棒性:目標 = 當前腳位置、目標在 pivot 上 ----
    Pose p2=start; solveCCD(sk,p2,FOOT,footPos(sk,start,FOOT),30);
    if(dist(footPos(sk,p2,FOOT),footPos(sk,start,FOOT))>1e-6){std::puts("FAIL: already-at-goal moved");++fails;}
    Pose p3=start; solveCCD(sk,p3,FOOT,w.position[HIP],30);   // 零長度向量情形
    for(auto& q:p3.localRotation) if(!finiteQ(q)){std::puts("FAIL: NaN at zero-length");++fails;}

    // ---- (D) 不該動的關節沒被改 ----
    if(out.localRotation[0].w!=1||out.localRotation[3].w!=1){std::puts("FAIL: hip/foot modified");++fails;}

    std::puts(fails? "IK CHECK FAILED":"IK CHECK PASSED");
    return fails?1:0;
}