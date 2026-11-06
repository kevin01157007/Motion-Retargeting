#include "motion.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int fails = 0;
static const double PI = 3.14159265358979323846;
#define CHECK(cond, msg) do { if(!(cond)){ std::printf("FAIL: %s\n", msg); ++fails; } \
                              else std::printf("ok:   %s\n", msg); } while(0)
static bool near(double a, double b, double e = 1e-9){ return std::fabs(a-b) < e; }
static bool nearV(Vec3 a, Vec3 b, double e = 1e-9){
    return near(a.x,b.x,e) && near(a.y,b.y,e) && near(a.z,b.z,e); }

static Skeleton makeSkel(){
    Skeleton s;
    s.joints = { {"root",-1,{0,1,0}}, {"spine",0,{0,0.5,0}}, {"arm",1,{1,0,0}} };
    return s;
}
static Pose identityPose(const Skeleton& s){
    Pose p; p.rootTranslation = {0,0,0};
    p.localRotation.assign(s.joints.size(), Quat{1,0,0,0});
    return p;
}
int main() {
    Skeleton s = makeSkel();
    Pose p = identityPose(s);
    WorldPose w = forwardKinematics(s, p);

    // 檢查一:rest pose
    CHECK(nearV(w.position[0], {0,1,0}),   "rest: root");
    CHECK(nearV(w.position[1], {0,1.5,0}), "rest: spine");
    CHECK(nearV(w.position[2], {1,1.5,0}), "rest: arm");

    // 檢查二:旋轉父關節
    p = identityPose(s);
    p.localRotation[1] = axisAngle({0,0,1}, PI/2);
    w = forwardKinematics(s, p);
    CHECK(nearV(w.position[1], {0,1.5,0}), "rot spine: spine itself stays");
    CHECK(nearV(w.position[2], {0,2.5,0}), "rot spine: arm moves to (0,2.5,0)");
    // root 平移
    p = identityPose(s);
    p.rootTranslation = {1, 2, 3};
    w = forwardKinematics(s, p);
    CHECK(nearV(w.position[0], {1,3,3}),     "translate: root");
    CHECK(nearV(w.position[2], {2,3.5,3}),   "translate: arm");
    CHECK(nearV(w.position[1], {1,3.5,3}), "translate: spine");

    // 乘法順序:root 繞 Y 90°,spine 繞 X 90°
    // Q[1] = Qy * Qx(先 X 再 Y)
    // P[1] = P[0] + rotate(Qy, (0,0.5,0)) = (0,1.5,0)(Y 軸不變)
    // P[2] = P[1] + rotate(Qy*Qx, (1,0,0))
    //   rotate(Qx,(1,0,0)) = (1,0,0);再 rotate(Qy,(1,0,0)) = (0,0,-1)
    //   => P[2] = (0, 1.5, -1)
    p = identityPose(s);
    p.localRotation[0] = axisAngle({0,1,0}, PI/2);
    p.localRotation[1] = axisAngle({1,0,0}, PI/2);
    w = forwardKinematics(s, p);
    CHECK(nearV(w.position[2], {0,1.5,-1}), "order: Qy*Qx");
    std::printf("%d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}