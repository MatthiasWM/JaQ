
#include "JQLeg.h"
#include "JQMisc.h"
#include "JQServo.h"


JQLeg::JQLeg(JQLegID id)
    :
    pID( id )
{
}

void JQLeg::linkTo(JQServo* hip, JQServo* knee, JQServo* ankle)
{
    pHip = hip;
    pKnee = knee;
    pAnkle = ankle;
}

void JQLeg::updatePositionFromAngles()
{
    double a = pHip->getAngle();
    double b = pKnee->getAngle();
    double c = pAnkle->getAngle();

    // take the foot and rotate it around the ankle
    double tx = -sin(c) * -111.4, tz = cos(c) * -111.4;

    // add the lower leg and rotate around the knee joint
    tz -= 111.4;
    double lx = cos(b) * tx - sin(b) * tz;
    double lz = cos(b) * tz + sin(b) * tx;

    // add the hip joint rotation
    double ly = -64.0;
    double sx = lx;
    double sy = cos(a) * ly + sin(a) * lz;
    double sz = cos(a) * lz - sin(a) * ly;

    if (pID==FRONT_LEFT || pID==BACK_LEFT) sy = -sy;

    pX = sx;
    pY = sy;
    pZ = sz;
}


/* Leg IK in 3 steps
 *
 * Step 1: calculate the length of the leg if viewed from the front (y/z)
 *
 * distanche from hip to toe  c = sqrt(y*y+z*z)
 * angle between c and z  alpha = atan2(y, z)
 * length of the leg in this plane depends on the upper leg length (a=64mm) and c
 * so length of leg  b = sqrt(c*c-a*a)
 * the angle between the upper leg a and the hip-toe c is beta = sin(b/c)
 *
 * so the hip angle is 90 - alpha - beta
 *
 * Step 2: calculate the ankle
 *
 * we use the length of the leg in (y/z), b, and take x into account
 * so the total length of the leg is  f = sqrt(b*b+x*x)
 * the ankle is derived from the length of the leg segments and the total leg length (g):
 * ankle gamma = acos( (f*f-g*g-g*g)/(-2*g*g) )
 *
 * Step 3: calculate the knee
 *
 * delta0 is the angle between the lower leg and f, and can be calculate
 * delta0 = acos( (g*g-f*f-g*g)/(-2*g*g) )
 * delta1 is the angle between the vertical and f
 * so delta1 = sin(z/f) ??
 * the knee is then simply  knee = delta0 - delta1
 */


void JQLeg::updateAnglesFromPosition()
{
    // FIXME: recognize undefined positions and set the angles to an acceptable value
    // FIXME: fix some flipping angles when z gets positive
    // FIXME: make motion linear by adjusting servo acceleration and speeds
    double x = pX, y = pY, z = pZ;
    if (pID == FRONT_LEFT || pID == BACK_LEFT) y = -y;

    // Calculate the hip servo angle
    // c is the distance from the hip to the toe in y/z
    double c = sqrt(y * y + z * z);
    // alpha1 is the angle between the horizontal axis and the vector hip-to-toe
    double alpha1 = atan2(z, -y);
    // alpha2 is the angle between the hip vector and hip-to-toe
    double alpha2 = triAngleFromSS90(64.0, c);

    // Calculate the ankle angle
    // get the length of knee-to-toe (d) in y/z
    double d = sqrt(c * c - 64.0 * 64.0);
    // calculate the length of knee-to-toe (e) in x/z
    double e = sqrt(d * d + x * x);
    // calculate the ankle using the length of the lower leg, the foot, and e
    double gamma = triAngleFromSSS(e, 111.4, 111.4);

    // Calculate the knee
    // get the angle between d and e
    double beta1 = atan2(-x, d);
    // get the angle between e and the lower leg
    double beta2 = (M_PI - gamma) / 2.0;

    pHip->setAngle(alpha1 + alpha2, false);
    pKnee->setAngle(-beta1 - beta2, false);
    pAnkle->setAngle(M_PI - gamma, false);
}

void JQLeg::setX(double v, bool propagate)
{
    pX = v;
    if (propagate)
        updateAnglesFromPosition();
}

void JQLeg::setY(double v, bool propagate)
{
    pY = v;
    if (propagate)
        updateAnglesFromPosition();
}

void JQLeg::setZ(double v, bool propagate)
{
    pZ = v;
    if (propagate)
        updateAnglesFromPosition();
}

