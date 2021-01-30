

#ifndef JQ_SERVO_H
#define JQ_SERVO_H

#include "JQMisc.h"

class JQLeg;


enum JQServoID {
    FL_HIP = 0, FL_KNEE, FL_ANKLE,
    FR_HIP, FR_KNEE, FR_ANKLE,
    BL_HIP, BL_KNEE, BL_ANKLE,
    BR_HIP, BR_KNEE, BR_ANKLE
};


class JQServo
{
public:
    JQServo(JQServoID id, uint32_t address);
    ~JQServo() = default;
    JQServoID ID() { return pID; }

    // manage the leg that contains this servo
    void linkTo(JQLeg*);
    void updateLegPosition();
    JQLeg* leg() { return pLeg; }

    // get and set angle in different systems (radians, degree, servo units)
    void setAngle(double rad, bool propagate = true);
    void setAngleDeg(double deg, bool propagate = true) { setAngle(deg2rad(deg), propagate); }
    void setAngleRaw(int32_t raw, bool propagate = true);
    double getAngle() const { return pAngle; }
    double getAngleDeg() const { return rad2deg(getAngle()); }
    int32_t getAngleRaw() const;

    // manage the relation to the servo hardware
    void setTrim(int32_t offset, int32_t direction);
    void sendAngle();
    int32_t requestAngle();
protected:
private:
    JQServoID pID;
    JQLeg* pLeg = nullptr;
    uint32_t pAddress = 0;
    double pAngle = 0.0;
    int32_t pTrimOffset = 0;
    int32_t pTrimDirection = 1;
    bool pChanged = true;
};

#endif
