#pragma once
#include <cmath>
#include <string>
#include <vector>

struct Vec3 { double x=0, y=0, z=0; };
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline double length(Vec3 a) { return std::sqrt(dot(a,a)); }

struct Quat { double w=1, x=0, y=0, z=0; };
Quat normalize(Quat q);           // TODO 1: math.cpp
Quat multiply(Quat a, Quat b);    // TODO 1: math.cpp
Quat axisAngle(Vec3 axis, double radians); // TODO 1: math.cpp
Vec3 rotate(Quat q, Vec3 v);     // TODO 1: math.cpp

struct Joint { std::string name; int parent=-1; Vec3 offset; };
struct Skeleton { std::vector<Joint> joints; };
struct Pose { Vec3 rootTranslation; std::vector<Quat> localRotation; };
struct Motion { Skeleton skeleton; std::vector<Pose> frames; double frameTime=0; };
struct WorldPose { std::vector<Vec3> position; std::vector<Quat> rotation; };

WorldPose forwardKinematics(const Skeleton& skeleton, const Pose& pose); // TODO 2
void solveCCD(const Skeleton& skeleton, Pose& pose, int endEffector,
              Vec3 target, int iterations); // TODO 3
Pose retargetFrame(const Skeleton& source, const Skeleton& target,
                   const Pose& sourcePose, double rootScale); // TODO 4
