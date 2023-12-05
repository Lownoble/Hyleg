#include "common/mathTypes.h"
#include "common/HyLeg.h"
#include <iostream>

int main(int argc, char *argv[])
{
    BipedalLeg* Leg = new HyLeg(0, Vec3(0, 0, 0));
    Vec3 pos;
    pos << 0.0019, 0, -0.46;
    Vec3 vel;
    vel << 0.7874, 0, 0.0021;
    Vec2 q = Leg->calcQ(pos);
    Mat2 J = Leg->calcJaco(q);
    Vec2 qd = Leg->calcQd(q, vel);
    printf("%f %f\n",J(0,0),J(0,1));
    printf("%f %f\n",J(1,0),J(1,1));
    printf("%f %f\n",qd(0),qd(1));
}