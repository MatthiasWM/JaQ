

#include "JQBot.h"
#include "JQLeg.h"
#include "JQServo.h"


JQBot::JQBot()
    :
    pLeg { 
        new JQLeg(FRONT_LEFT), new JQLeg(FRONT_RIGHT), new JQLeg(BACK_LEFT), new JQLeg(BACK_RIGHT)
    },
    pServo { 
        new JQServo(FL_HIP, 10), new JQServo(FL_KNEE, 6), new JQServo(FL_ANKLE, 2),
        new JQServo(FR_HIP, 12), new JQServo(FR_KNEE, 8), new JQServo(FR_ANKLE, 4),
        new JQServo(BL_HIP, 11), new JQServo(BL_KNEE, 7), new JQServo(BL_ANKLE, 3),
        new JQServo(BR_HIP,  9), new JQServo(BR_KNEE, 5), new JQServo(BR_ANKLE, 1)
    }
{
    pLeg[FRONT_LEFT]->linkTo(pServo[FL_HIP], pServo[FL_KNEE], pServo[FL_ANKLE]);
    pServo[FL_HIP]->linkTo(pLeg[FRONT_LEFT]);
    pServo[FL_KNEE]->linkTo(pLeg[FRONT_LEFT]);
    pServo[FL_ANKLE]->linkTo(pLeg[FRONT_LEFT]);
    pLeg[FRONT_RIGHT]->linkTo(pServo[FR_HIP], pServo[FR_KNEE], pServo[FR_ANKLE]);
    pServo[FR_HIP]->linkTo(pLeg[FRONT_RIGHT]);
    pServo[FR_KNEE]->linkTo(pLeg[FRONT_RIGHT]);
    pServo[FR_ANKLE]->linkTo(pLeg[FRONT_RIGHT]);
    pLeg[BACK_LEFT]->linkTo(pServo[BL_HIP], pServo[BL_KNEE], pServo[BL_ANKLE]);
    pServo[BL_HIP]->linkTo(pLeg[BACK_LEFT]);
    pServo[BL_KNEE]->linkTo(pLeg[BACK_LEFT]);
    pServo[BL_ANKLE]->linkTo(pLeg[BACK_LEFT]);
    pLeg[BACK_RIGHT]->linkTo(pServo[BR_HIP], pServo[BR_KNEE], pServo[BR_ANKLE]);
    pServo[BR_HIP]->linkTo(pLeg[BACK_RIGHT]);
    pServo[BR_KNEE]->linkTo(pLeg[BACK_RIGHT]);
    pServo[BR_ANKLE]->linkTo(pLeg[BACK_RIGHT]);
}


