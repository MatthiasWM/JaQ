

#include "JQServo.h"
#include "JQMisc.h"
#include "JQLeg.h"
#include "SCServo.h"


extern SCServo sc;


JQServo::JQServo(JQServoID id, uint32_t address)
    :
    pID( id ),
    pAddress( address )
{
    static int32_t trimOffsetLUT[] = { 0, 1240, 2839, 2885, 1263, 1193, 2951, 3025, 1142, 2074, 2089, 2071, 2048 };
    static int32_t trimDirectionLUT[] = { 0, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1 };
    static double angleLUT[] = { 0.0, 90.0, 90.0, 90.0, 90.0, -45.0, -45.0, -45.0, -45.0, 0.0, 0.0, 0.0, 0.0 };
    pTrimOffset = trimOffsetLUT[address];
    pTrimDirection = trimDirectionLUT[address];
    pAngle = deg2rad(angleLUT[address]);
}

void JQServo::linkTo(JQLeg* leg)
{
    pLeg = leg;
}

void JQServo::updateLegPosition()
{
    pLeg->updatePositionFromAngles();
}

void JQServo::setAngle(double rad, bool propagate)
{
    if (rad != pAngle) {
        pAngle = rad;
        pChanged = true;
    }
    if (propagate)
        updateLegPosition();
}

void JQServo::setAngleRaw(int32_t raw, bool propagate)
{
    setAngle(((raw - pTrimOffset) * pTrimDirection) / 2048.0 * M_PI, propagate);
}

int32_t JQServo::getAngleRaw() const
{
    return ((int32_t)(getAngle() / M_PI * 2048.0)) * pTrimDirection + pTrimOffset;
}


void JQServo::setTrim(int32_t offset, int32_t direction)
{
    pTrimOffset = offset;
    pTrimDirection = direction;
    pChanged = true;
}


void JQServo::sendAngle()
{
    if (pChanged && sc.isOpen()) {
        sc.writeWord(pAddress, 42, getAngleRaw());
        pChanged = false;
    }
}


int32_t JQServo::requestAngle()
{
    if (sc.isOpen())
        return sc.ReadPos(pAddress);
    else
        return -1;
}


