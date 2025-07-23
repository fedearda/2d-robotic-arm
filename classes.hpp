#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

// The rationale behind the verbosity is the following:
// SILENT: lowest verbosity -> you only read the result and what went wrong
// INFO: you read the actions you would see in a physical system and the actions at high level of the algorithm
// DEBUG: all calculations in the algorithms relevant to ccd are logged
enum Verbosity{
    SILENT = 1,
    INFO = 2,
    DEBUG = 3
};

using namespace std;

const double THRESH = 5e-3;
const float MAX_ANGLE_PER_STEP = 0.15; 
const double PI = 3.141592653589793;
const int  MAX_ITERATIONS = 500;
const float DAMP_THRESHOLD = 0.2;
const float DAMP_FACTOR = 0.5;
const Verbosity VERBOSITY_LEVEL = INFO;

void log(Verbosity lvl, string msg){
    if (lvl <= VERBOSITY_LEVEL) {
        std::cout << msg << std::endl;
    }
}

struct Vector2 {
    double x;
    double y;
    public:
    Vector2(double xi, double yi):x(xi),y(yi){}
    Vector2 operator +(const Vector2& other) const{
        return Vector2(x+other.x,y+other.y);
    }
    Vector2 operator -(const Vector2& other) const{
        return Vector2(x-other.x,y-other.y);
    }
};

float angleBetween(Vector2 a, Vector2 b){
    return atan2(a.x*b.y - a.y*b.x , a.x*b.x + a.y*b.y); // returns angles [-pi,pi]
}

float computeDist(Vector2 a, Vector2 b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

float module(Vector2 v){
        return sqrt(v.x*v.x+v.y*v.y);
}

void normalize(Vector2 * v){
        v->x /= module(*v);
        v->y /= module(*v);
    }

bool isTargetReached(Vector2 endEff, Vector2 target, float threshold){
    return (computeDist(endEff, target)<threshold);
}

class Joint {
    float angle; //angle wrt the horizontal axis
    float length;
    Vector2 position; //this is the coordinate of the point closer to the root of the arm
    public:
        Joint(float a, float l, Vector2 pos):angle(a),length(l),position(pos){}
        float getLength(){return length;}
        float getAngle(){return angle;}
        Vector2 getPos(){return position;}
        void setPos(Vector2 new_pos){position = new_pos;};
        void rotateByAngle(float ang){angle+=ang;};
};

Joint buildFromPrevious(Joint prev_j, float ang, float l){
    Joint res(ang,l,{prev_j.getPos().x + prev_j.getLength()*cos(prev_j.getAngle()),prev_j.getPos().y + prev_j.getLength()*sin(prev_j.getAngle())});
    return res;
}

Vector2 attachEndEffectorFromJoint(Joint j){
    Vector2 ee { j.getPos().x + j.getLength() * cos(j.getAngle()) , j.getPos().y + j.getLength() * sin(j.getAngle()) };
    return ee;
}

class Arm {
    std::vector<Joint> joints;
    Vector2 endEffector;
    public:
        Arm(std::vector<Joint> j, Vector2 eE):joints(j),endEffector(eE){}
        Vector2 getEndEffector(){return endEffector;}
        void setEndEffector(Vector2 eE){endEffector = eE;}
        void rotateToTarget(Vector2 target);
        void updateForwardKinematics();
        bool isTargetReachable(Vector2 target){
            return (computeDist(target,joints[0].getPos()) <= fullLength());
        }
        float fullLength(){
            float sum = 0.0;
            for (auto e : joints){
                sum += e.getLength();
            }
            return sum;
        }
};