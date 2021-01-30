



#ifndef JQ_LEG_H
#define JQ_LEG_H

#include <stdint.h>

class JQServo;


enum JQLegID {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};


class JQLeg
{
public:
    JQLeg(JQLegID);
    ~JQLeg() = default;
    JQLegID ID() { return pID; }
    void linkTo(JQServo* hip, JQServo* knee, JQServo* ankle);
    void updatePositionFromAngles();
    void updateAnglesFromPosition();
    double getX() { return pX; }
    double getY() { return pY; }
    double getZ() { return pZ; }
    void setX(double v, bool propagate = true);
    void setY(double v, bool propagate = true);
    void setZ(double v, bool propagate = true);
    void setPosition(double x, double y, double z, bool propagate = true);
    JQServo* getHipServo() const { return pHip; }
    JQServo* getKneeServo() const { return pKnee; }
    JQServo* getAnkleServo() const { return pAnkle; }
protected:
    JQLegID pID;
    JQServo* pHip = nullptr;
    JQServo* pKnee = nullptr;
    JQServo* pAnkle = nullptr;
    double pX = 0.0;
    double pY = 0.0;
    double pZ = 0.0;
};

#endif
