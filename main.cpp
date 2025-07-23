#include "classes.hpp"
#include "ccd.hpp"

int main(){
    Joint j1 {0.0,0.5,{0,0}};
    Joint j2 = buildFromPrevious(j1,0.0,0.5);
    Joint j3 = buildFromPrevious(j2,0.0,0.5);
    Joint j4 = buildFromPrevious(j3,0.0,0.5);
    Vector2 hand = attachEndEffectorFromJoint(j4);
    cout << "** EndEffector is at position ("<< hand.x << "," << hand.y <<") **" << endl;
    Arm A {{j1,j2,j3,j4},hand};
    Vector2 targ {1.5, 0.1};
    solveInverseKin_CCD(A,targ);
}