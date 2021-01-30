

#ifndef JQ_BOT_H
#define JQ_BOT_H

#include "JQMisc.h"

class JQLeg;
class JQServo;

enum JQLegID;
enum JQServoID;



class JQBot
{
public:
    JQBot();
    ~JQBot() = default;
    JQLeg& leg(JQLegID id) { return *pLeg[id]; }
    JQServo& servo(JQServoID id) { return *pServo[id]; }
protected:
private:
    JQLeg* pLeg[4];
    JQServo* pServo[12];
};


#endif
