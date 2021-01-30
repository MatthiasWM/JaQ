// Jaq.cpp : Defines the entry point for the application.
//

/* TODO:
 *
 * + implement RAW to Angle and Angle to Raw
 * - implement Angle to Position (Forward Kinematics)
 * - implement Position to Angle (Inverse Kinematics)
 * - add manual mode
 * - add full servo settings and emergency shutoff
 * - add keyframes
 * - add keyframe interpolation
 * - add keyframe sequences
 * - add IK and UI to move and rotate body
 * - move everything into the robot and add a remote control system (battery, DC-DC converter, switches, display, charging)
 * - add communication to rotational sensor
 * - add motor current feeedback
 * - add balance
 * - add task-based motion
 * - add reactive motion and walk cycle
 * - add vision
 * - add surface recognition and adapted walking
 * - add target recognintion
 * - add path planning
 * ...
 */

#include "Jaq.h"

#include "JaqUI.h"
#include "JQServo.h"
#include "JQLeg.h"
#include "JQBot.h"

#include "SCServo.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <FL/Fl_Widget.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Simple_Terminal.h>
#include <FL/Fl_Preferences.H>
#include <FL/fl_ask.h>

using namespace std;

JQBot gBot;
SCServo sc;

int gAppWindowX = 0x7fffffff;
int gAppWindowY = 0x7fffffff;


// ---- Preferences -----------------------------------------------------------
/**
 * Load calibration data and other preferences.
 * This function does not update the UI.
 */
void loadCalibration()
{
#if 0
    Fl_Preferences prefs(Fl_Preferences::USER, "matthiasm.com", "Jaq");
    Fl_Preferences ui(prefs, "UI");
    Fl_Preferences appWindow(ui, "appWindow");
    appWindow.get("x", gAppWindowX, 0x7fffffff);
    appWindow.get("y", gAppWindowY, 0x7fffffff);
    //appWindow.get("w", gAppWindowW, 0x7fffffff);
    //appWindow.get("h", gAppWindowH, 0x7fffffff);
    Fl_Preferences calibration(prefs, "calibration");
    for (int i = 1; i <= 12; i++) {
        calibration.get(Fl_Preferences::Name(i), gServoCal[i], 0x7fffffff);
        //gServo[i].setTrim(gServoCal[i], gServoDir[i]);
    }
#endif
}

/**
 * Save calibration data to the preferences database.
 * This function does not update the UI.
 */
void writeCalibration()
{
#if 0
    Fl_Preferences prefs(Fl_Preferences::USER, "matthiasm.com", "Jaq");
    Fl_Preferences calibration(prefs, "calibration");
    for (int i = 1; i <= 12; i++) {
        calibration.set(Fl_Preferences::Name(i), gServoCal[i]);
    }
#endif
}


void updateLegPositionUI(JQLeg* leg)
{
    int32_t ix = (int32_t)leg->ID() * 3;
    wPosition[ix]->value(leg->getX());
    wPosition[ix + 1]->value(leg->getY());
    wPosition[ix + 2]->value(leg->getZ());
}

void updateLegAngleUI(JQLeg* leg)
{
    JQServo* servo;
    servo = leg->getHipServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
    servo = leg->getKneeServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
    servo = leg->getAnkleServo();
    wRawAngle[servo->ID()]->value(servo->getAngleRaw());
    wAngle[servo->ID()]->value(servo->getAngleDeg());
}



// ---- Callbacks -------------------------------------------------------------

void quitCB(Fl_Widget *w, void*)
{
    closeComCB(nullptr, nullptr);
    w->hide();
}


void openComCB(Fl_Button*, void*)
{
    sc.End = 0;
    int ret = sc.open(wComName->value());
    char buf[32];
    sprintf(buf, "OpenCom: %d\n", ret);
    wTerminal->append(buf);
    ret = sc.Ping(1);
    sprintf(buf, "Ping 1: %d\n", ret);
    wTerminal->append(buf);
//    ret = sc.Ping(100);
//    sprintf(buf, "Ping 100: %d\n", ret);
//    wTerminal->append(buf);
    Sleep(100);
    testComCB(nullptr, nullptr);
    Sleep(100);
    servosPowerOnCB(nullptr, nullptr);
}

void servosPowerOnCB(Fl_Button*, void*)
{
    for (int i = 1; i <= 12; i++) {
        sc.EnableTorque(i, 1);
    }
}

void servosPowerOffCB(Fl_Button*, void*)
{
    for (int i = 1; i <= 12; i++) {
        sc.EnableTorque(i, 0);
    }
}

void testComCB(Fl_Button*, void*)
{
    if (!sc.isOpen())
        return;
#if 0
    sc.EnableTorque(1, 1);
    Sleep(10);
    sc.writeWord(1, 42, 1000);
    Sleep(1000);
    sc.writeWord(1, 42, 2000);
#endif
    // read all servo positions and fill the corresponding sliders with that value
    for (int i = 0; i < 12; i++) {
        JQServo& servo = gBot.servo((JQServoID)i);
        int pos = servo.requestAngle();
        if (pos != -1) {
            servo.setAngleRaw(pos, false);
        }
    }

    // update all sliders based on the values read from the servos
    for (int i = 0; i < 4; i++) {
        JQLeg& leg = gBot.leg((JQLegID)i);
        leg.updatePositionFromAngles();
        updateLegAngleUI(&leg);
        updateLegPositionUI(&leg);
    }
}

void closeComCB(Fl_Button*, void*)
{
    if (sc.isOpen()) {
        servosPowerOffCB(nullptr, nullptr);
        Sleep(100);
        sc.close();
    }
}


void calibrateCB(Fl_Button*, void*)
{
#if 0
    const int DEG90 = (int)(90.0 / 360.0 * 4096);
    const int DEG45 = (int)(45.0 / 360.0 * 4096);
    static int preset[13] = { 0,  DEG90, DEG90, DEG90, DEG90,  -DEG45, -DEG45, -DEG45, -DEG45,  0, 0, 0, 0 };

    const char *code = fl_input("This will overwrite previous calibrations.\nPlease enter calibration security code.\n(Hint: it's 9999)");
    if (code && strcmp(code, "9999") == 0) {
        for (int i = 1; i <= 12; i++) {
            int raw = (int)wRawAngle[i]->value();
            gServoCal[i] = raw - gServoDir[i] * preset[i];
            //wAngle[i]->value((wRawAngle[i]->value() - gServoCal[i]) * gServoDir[i] / 4096.0 * 360.0);
        }
        // store new calibration data in database
        writeCalibration();
        // update all vanilla sliders based on raw values and calibration
        rawToVanilla();
        // update all positions based on the vanilla sliders
        angleToPosition();
        fl_message("Calibration overwritten");
    }
#endif
}

void setRawAngleCB(Fl_Value_Slider* w, long ix)
{
    JQServo &servo = gBot.servo((JQServoID)ix);

    // update the servo (also calculates AngleRad and Leg Positions)
    servo.setAngleRaw((int32_t)w->value());

    // update the UI
    wAngle[servo.ID()]->value(servo.getAngleDeg());
    updateLegPositionUI(servo.leg());

    servo.sendAngle();
}

void setAngleCB(Fl_Value_Slider* w, long ix)
{
    JQServo& servo = gBot.servo((JQServoID)ix);

    // update the servo (also calculates AngleRad and Leg Positions)
    servo.setAngleDeg(w->value());

    // update the UI
    wRawAngle[servo.ID()]->value(servo.getAngleRaw());
    updateLegPositionUI(servo.leg());

    servo.sendAngle();
}

void setPositionCB(Fl_Value_Slider* w, long ix)
{
    JQLeg& leg = gBot.leg(JQLegID(ix / 3));

    // update the leg (also calculates all angles)
    switch (ix % 3) {
    case 0: leg.setX(w->value()); break;
    case 1: leg.setY(w->value()); break;
    case 2: leg.setZ(w->value()); wAllZ->value(w->value()); break;
    }

    // update the UI
    updateLegAngleUI(&leg);

    // update servo positions
    leg.getHipServo()->sendAngle();
    leg.getKneeServo()->sendAngle();
    leg.getAnkleServo()->sendAngle();
}

void setAllZCB(Fl_Value_Slider* w, void*)
{
    double z = w->value();
    uint32_t t0 = GetTickCount();
    wPosition[2]->value(z);
    wPosition[2]->do_callback();
    wPosition[5]->value(z);
    wPosition[5]->do_callback();
    wPosition[8]->value(z);
    wPosition[8]->do_callback();
    wPosition[11]->value(z);
    wPosition[11]->do_callback();
    uint32_t t1 = GetTickCount();
    wTerminal->printf("That took %dms.\n", t1 - t0);
#if 0
    int ix;
    for (ix = 1; ix <= 4; ix++) {
        wPosition[ix]->value(z);
    }
    positionToAngle();
    vanillaToRaw();
    uint32_t t0 = GetTickCount();
    for (ix = 1; ix <= 4; ix++) {
        sc.RegWritePos(ix, wRawAngle[ix]->value(), 0, 0);
        sc.RegWritePos(ix+4, wRawAngle[ix+4]->value(), 0, 0);
        sc.RegWritePos(ix+8, wRawAngle[ix+8]->value(), 0, 0);
        //sc.writeWord(ix, 42, wRawAngle[ix]->value());
        //sc.writeWord(ix + 4, 42, wRawAngle[ix + 4]->value());
        //sc.writeWord(ix + 8, 42, wRawAngle[ix + 8]->value());
    }
    sc.RegWriteAction();
    uint32_t t1 = GetTickCount();
    wTerminal->printf("That took %dms.\n", t1 - t0);
#endif
    Fl::flush(); // FIXME: ouch!
}

int main(int argc, char** argv)
{
    Fl_Window* win = createMainWindow();
    if (gAppWindowX != 0x7fffffff) win->position(gAppWindowX, gAppWindowY);

    // load calibration data from last session
    //loadCalibration();

    for (int i = 0; i < 4; i++) {
        JQLeg& leg = gBot.leg((JQLegID)i);
        leg.updatePositionFromAngles();
        updateLegAngleUI(&leg);
        updateLegPositionUI(&leg);
    }

    win->show(argc, argv);
    win->callback(quitCB);
    wTerminal->append("Welcome to Jaq\n");
    Fl::lock();
    Fl::run();
    return 0;
}
