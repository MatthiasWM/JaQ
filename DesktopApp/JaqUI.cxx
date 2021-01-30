// generated by Fast Light User Interface Designer (fluid) version 1.0400

#include "JaqUI.h"

Fl_Input *wComName=(Fl_Input *)0;

Fl_Simple_Terminal *wTerminal=(Fl_Simple_Terminal *)0;

Fl_Value_Slider *wRawAngle[12]={(Fl_Value_Slider *)0};

Fl_Value_Slider *wAngle[12]={(Fl_Value_Slider *)0};

Fl_Value_Slider *wPosition[12]={(Fl_Value_Slider *)0};

Fl_Value_Slider *wAllZ=(Fl_Value_Slider *)0;

Fl_Double_Window* createMainWindow() {
  Fl_Double_Window* w;
  { Fl_Double_Window* o = new Fl_Double_Window(650, 780, "Jaq - Just Another Quadruped");
    w = o; if (w) {/* empty */}
    { Fl_Button* o = new Fl_Button(180, 10, 75, 25, "Open");
      o->callback((Fl_Callback*)openComCB);
    } // Fl_Button* o
    { wComName = new Fl_Input(80, 10, 100, 25, "Com Port:");
    } // Fl_Input* wComName
    { Fl_Button* o = new Fl_Button(255, 10, 75, 25, "Test");
      o->callback((Fl_Callback*)testComCB);
    } // Fl_Button* o
    { Fl_Button* o = new Fl_Button(330, 10, 75, 25, "Close");
      o->callback((Fl_Callback*)closeComCB);
    } // Fl_Button* o
    { wTerminal = new Fl_Simple_Terminal(5, 700, 635, 75);
      wTerminal->box(FL_DOWN_FRAME);
      wTerminal->color(FL_GRAY0);
      wTerminal->selection_color((Fl_Color)10);
      wTerminal->labeltype(FL_NORMAL_LABEL);
      wTerminal->labelfont(5);
      wTerminal->labelsize(11);
      wTerminal->labelcolor(FL_FOREGROUND_COLOR);
      wTerminal->textfont(5);
      wTerminal->textsize(11);
      wTerminal->textcolor((Fl_Color)2);
      wTerminal->align(Fl_Align(FL_ALIGN_TOP));
      wTerminal->when(FL_WHEN_RELEASE);
    } // Fl_Simple_Terminal* wTerminal
    { Fl_Button* o = new Fl_Button(405, 10, 75, 25, "Power");
      o->callback((Fl_Callback*)servosPowerOnCB);
    } // Fl_Button* o
    { Fl_Button* o = new Fl_Button(480, 10, 75, 25, "Power Off");
      o->callback((Fl_Callback*)servosPowerOffCB);
    } // Fl_Button* o
    { Fl_Tabs* o = new Fl_Tabs(0, 45, 650, 295);
      { Fl_Group* o = new Fl_Group(0, 70, 650, 270, "RAW");
        { wRawAngle[0] = new Fl_Value_Slider(50, 90, 270, 20, "Hip");
          wRawAngle[0]->type(1);
          wRawAngle[0]->labelsize(12);
          wRawAngle[0]->maximum(4095);
          wRawAngle[0]->step(1);
          wRawAngle[0]->callback((Fl_Callback*)setRawAngleCB, (void*)(FL_HIP));
          wRawAngle[0]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[0]
        { wRawAngle[1] = new Fl_Value_Slider(50, 110, 270, 20, "Knee");
          wRawAngle[1]->type(1);
          wRawAngle[1]->labelsize(12);
          wRawAngle[1]->maximum(4095);
          wRawAngle[1]->step(1);
          wRawAngle[1]->callback((Fl_Callback*)setRawAngleCB, (void*)(FL_KNEE));
          wRawAngle[1]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[1]
        { wRawAngle[2] = new Fl_Value_Slider(50, 130, 270, 20, "Ankle");
          wRawAngle[2]->type(1);
          wRawAngle[2]->labelsize(12);
          wRawAngle[2]->maximum(4095);
          wRawAngle[2]->step(1);
          wRawAngle[2]->callback((Fl_Callback*)setRawAngleCB, (void*)(FL_ANKLE));
          wRawAngle[2]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[2]
        { wRawAngle[3] = new Fl_Value_Slider(360, 90, 270, 20, "Hip");
          wRawAngle[3]->type(1);
          wRawAngle[3]->labelsize(12);
          wRawAngle[3]->maximum(4095);
          wRawAngle[3]->step(1);
          wRawAngle[3]->callback((Fl_Callback*)setRawAngleCB, (void*)(FR_HIP));
          wRawAngle[3]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[3]
        { wRawAngle[4] = new Fl_Value_Slider(360, 110, 270, 20, "Knee");
          wRawAngle[4]->type(1);
          wRawAngle[4]->labelsize(12);
          wRawAngle[4]->maximum(4095);
          wRawAngle[4]->step(1);
          wRawAngle[4]->callback((Fl_Callback*)setRawAngleCB, (void*)(FR_KNEE));
          wRawAngle[4]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[4]
        { wRawAngle[5] = new Fl_Value_Slider(360, 130, 270, 20, "Ankle");
          wRawAngle[5]->type(1);
          wRawAngle[5]->labelsize(12);
          wRawAngle[5]->maximum(4095);
          wRawAngle[5]->step(1);
          wRawAngle[5]->callback((Fl_Callback*)setRawAngleCB, (void*)(FR_ANKLE));
          wRawAngle[5]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[5]
        { wRawAngle[6] = new Fl_Value_Slider(50, 170, 270, 20, "Hip");
          wRawAngle[6]->type(1);
          wRawAngle[6]->labelsize(12);
          wRawAngle[6]->maximum(4095);
          wRawAngle[6]->step(1);
          wRawAngle[6]->callback((Fl_Callback*)setRawAngleCB, (void*)(BL_HIP));
          wRawAngle[6]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[6]
        { wRawAngle[7] = new Fl_Value_Slider(50, 190, 270, 20, "Knee");
          wRawAngle[7]->type(1);
          wRawAngle[7]->labelsize(12);
          wRawAngle[7]->maximum(4095);
          wRawAngle[7]->step(1);
          wRawAngle[7]->callback((Fl_Callback*)setRawAngleCB, (void*)(BL_KNEE));
          wRawAngle[7]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[7]
        { wRawAngle[8] = new Fl_Value_Slider(50, 210, 270, 20, "Ankle");
          wRawAngle[8]->type(1);
          wRawAngle[8]->labelsize(12);
          wRawAngle[8]->maximum(4095);
          wRawAngle[8]->step(1);
          wRawAngle[8]->callback((Fl_Callback*)setRawAngleCB, (void*)(BL_ANKLE));
          wRawAngle[8]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[8]
        { wRawAngle[9] = new Fl_Value_Slider(360, 170, 270, 20, "Hip");
          wRawAngle[9]->type(1);
          wRawAngle[9]->labelsize(12);
          wRawAngle[9]->maximum(4095);
          wRawAngle[9]->step(1);
          wRawAngle[9]->callback((Fl_Callback*)setRawAngleCB, (void*)(BR_HIP));
          wRawAngle[9]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[9]
        { wRawAngle[10] = new Fl_Value_Slider(360, 190, 270, 20, "Knee");
          wRawAngle[10]->type(1);
          wRawAngle[10]->labelsize(12);
          wRawAngle[10]->maximum(4095);
          wRawAngle[10]->step(1);
          wRawAngle[10]->callback((Fl_Callback*)setRawAngleCB, (void*)(BR_KNEE));
          wRawAngle[10]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[10]
        { wRawAngle[11] = new Fl_Value_Slider(360, 210, 270, 20, "Ankle");
          wRawAngle[11]->type(1);
          wRawAngle[11]->labelsize(12);
          wRawAngle[11]->maximum(4095);
          wRawAngle[11]->step(1);
          wRawAngle[11]->callback((Fl_Callback*)setRawAngleCB, (void*)(BR_ANKLE));
          wRawAngle[11]->align(Fl_Align(FL_ALIGN_LEFT));
        } // Fl_Value_Slider* wRawAngle[11]
        { Fl_Button* o = new Fl_Button(525, 250, 105, 25, "Calibrate");
          o->callback((Fl_Callback*)calibrateCB);
        } // Fl_Button* o
        o->end();
      } // Fl_Group* o
      { Fl_Group* o = new Fl_Group(0, 70, 650, 270, "Angles");
        o->hide();
        o->end();
      } // Fl_Group* o
      { Fl_Group* o = new Fl_Group(0, 70, 650, 270, "Position");
        o->hide();
        o->end();
      } // Fl_Group* o
      o->end();
    } // Fl_Tabs* o
    { wAngle[0] = new Fl_Value_Slider(55, 355, 270, 20, "Hip");
      wAngle[0]->type(1);
      wAngle[0]->labelsize(12);
      wAngle[0]->minimum(-90);
      wAngle[0]->maximum(90);
      wAngle[0]->step(1);
      wAngle[0]->callback((Fl_Callback*)setAngleCB, (void*)(FL_HIP));
      wAngle[0]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[0]
    { wAngle[1] = new Fl_Value_Slider(55, 375, 270, 20, "Knee");
      wAngle[1]->type(1);
      wAngle[1]->labelsize(12);
      wAngle[1]->minimum(-180);
      wAngle[1]->maximum(90);
      wAngle[1]->step(1);
      wAngle[1]->value(-45);
      wAngle[1]->callback((Fl_Callback*)setAngleCB, (void*)(FL_KNEE));
      wAngle[1]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[1]
    { wAngle[2] = new Fl_Value_Slider(55, 395, 270, 20, "Ankle");
      wAngle[2]->type(1);
      wAngle[2]->labelsize(12);
      wAngle[2]->maximum(163);
      wAngle[2]->step(1);
      wAngle[2]->value(90);
      wAngle[2]->callback((Fl_Callback*)setAngleCB, (void*)(FL_ANKLE));
      wAngle[2]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[2]
    { wAngle[3] = new Fl_Value_Slider(365, 355, 270, 20, "Hip");
      wAngle[3]->type(1);
      wAngle[3]->labelsize(12);
      wAngle[3]->minimum(-90);
      wAngle[3]->maximum(90);
      wAngle[3]->step(1);
      wAngle[3]->callback((Fl_Callback*)setAngleCB, (void*)(FR_HIP));
      wAngle[3]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[3]
    { wAngle[4] = new Fl_Value_Slider(365, 375, 270, 20, "Knee");
      wAngle[4]->type(1);
      wAngle[4]->labelsize(12);
      wAngle[4]->minimum(-180);
      wAngle[4]->maximum(90);
      wAngle[4]->step(1);
      wAngle[4]->value(-45);
      wAngle[4]->callback((Fl_Callback*)setAngleCB, (void*)(FR_KNEE));
      wAngle[4]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[4]
    { wAngle[5] = new Fl_Value_Slider(365, 395, 270, 20, "Ankle");
      wAngle[5]->type(1);
      wAngle[5]->labelsize(12);
      wAngle[5]->maximum(163);
      wAngle[5]->step(1);
      wAngle[5]->value(90);
      wAngle[5]->callback((Fl_Callback*)setAngleCB, (void*)(FR_ANKLE));
      wAngle[5]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[5]
    { wAngle[6] = new Fl_Value_Slider(55, 435, 270, 20, "Hip");
      wAngle[6]->type(1);
      wAngle[6]->labelsize(12);
      wAngle[6]->minimum(-90);
      wAngle[6]->maximum(90);
      wAngle[6]->step(1);
      wAngle[6]->callback((Fl_Callback*)setAngleCB, (void*)(BL_HIP));
      wAngle[6]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[6]
    { wAngle[7] = new Fl_Value_Slider(55, 455, 270, 20, "Knee");
      wAngle[7]->type(1);
      wAngle[7]->labelsize(12);
      wAngle[7]->minimum(-180);
      wAngle[7]->maximum(90);
      wAngle[7]->step(1);
      wAngle[7]->value(-45);
      wAngle[7]->callback((Fl_Callback*)setAngleCB, (void*)(BL_KNEE));
      wAngle[7]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[7]
    { wAngle[8] = new Fl_Value_Slider(55, 475, 270, 20, "Ankle");
      wAngle[8]->type(1);
      wAngle[8]->labelsize(12);
      wAngle[8]->maximum(163);
      wAngle[8]->step(1);
      wAngle[8]->value(90);
      wAngle[8]->callback((Fl_Callback*)setAngleCB, (void*)(BL_ANKLE));
      wAngle[8]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[8]
    { wAngle[9] = new Fl_Value_Slider(365, 435, 270, 20, "Hip");
      wAngle[9]->type(1);
      wAngle[9]->labelsize(12);
      wAngle[9]->minimum(-90);
      wAngle[9]->maximum(90);
      wAngle[9]->step(1);
      wAngle[9]->callback((Fl_Callback*)setAngleCB, (void*)(BR_HIP));
      wAngle[9]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[9]
    { wAngle[10] = new Fl_Value_Slider(365, 455, 270, 20, "Knee");
      wAngle[10]->type(1);
      wAngle[10]->labelsize(12);
      wAngle[10]->minimum(-180);
      wAngle[10]->maximum(90);
      wAngle[10]->step(1);
      wAngle[10]->value(-45);
      wAngle[10]->callback((Fl_Callback*)setAngleCB, (void*)(BR_KNEE));
      wAngle[10]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[10]
    { wAngle[11] = new Fl_Value_Slider(365, 475, 270, 20, "Ankle");
      wAngle[11]->type(1);
      wAngle[11]->labelsize(12);
      wAngle[11]->maximum(163);
      wAngle[11]->step(1);
      wAngle[11]->value(90);
      wAngle[11]->callback((Fl_Callback*)setAngleCB, (void*)(BR_ANKLE));
      wAngle[11]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAngle[11]
    { wPosition[0] = new Fl_Value_Slider(55, 510, 270, 20, "X");
      wPosition[0]->type(1);
      wPosition[0]->labelsize(12);
      wPosition[0]->minimum(-250);
      wPosition[0]->maximum(250);
      wPosition[0]->step(1);
      wPosition[0]->callback((Fl_Callback*)setPositionCB, (void*)(0));
      wPosition[0]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[0]
    { wPosition[1] = new Fl_Value_Slider(55, 530, 270, 20, "Y");
      wPosition[1]->type(1);
      wPosition[1]->labelsize(12);
      wPosition[1]->minimum(-250);
      wPosition[1]->maximum(250);
      wPosition[1]->step(1);
      wPosition[1]->callback((Fl_Callback*)setPositionCB, (void*)(1));
      wPosition[1]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[1]
    { wPosition[2] = new Fl_Value_Slider(55, 550, 270, 20, "Z");
      wPosition[2]->type(1);
      wPosition[2]->labelsize(12);
      wPosition[2]->minimum(-250);
      wPosition[2]->maximum(250);
      wPosition[2]->step(1);
      wPosition[2]->value(-157);
      wPosition[2]->callback((Fl_Callback*)setPositionCB, (void*)(2));
      wPosition[2]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[2]
    { wPosition[3] = new Fl_Value_Slider(365, 510, 270, 20, "X");
      wPosition[3]->type(1);
      wPosition[3]->labelsize(12);
      wPosition[3]->minimum(-250);
      wPosition[3]->maximum(250);
      wPosition[3]->step(1);
      wPosition[3]->callback((Fl_Callback*)setPositionCB, (void*)(3));
      wPosition[3]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[3]
    { wPosition[4] = new Fl_Value_Slider(365, 530, 270, 20, "Y");
      wPosition[4]->type(1);
      wPosition[4]->labelsize(12);
      wPosition[4]->minimum(-250);
      wPosition[4]->maximum(250);
      wPosition[4]->step(1);
      wPosition[4]->callback((Fl_Callback*)setPositionCB, (void*)(4));
      wPosition[4]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[4]
    { wPosition[5] = new Fl_Value_Slider(365, 550, 270, 20, "Z");
      wPosition[5]->type(1);
      wPosition[5]->labelsize(12);
      wPosition[5]->minimum(-250);
      wPosition[5]->maximum(250);
      wPosition[5]->step(1);
      wPosition[5]->value(-157);
      wPosition[5]->callback((Fl_Callback*)setPositionCB, (void*)(5));
      wPosition[5]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[5]
    { wPosition[6] = new Fl_Value_Slider(55, 590, 270, 20, "X");
      wPosition[6]->type(1);
      wPosition[6]->labelsize(12);
      wPosition[6]->minimum(-250);
      wPosition[6]->maximum(250);
      wPosition[6]->step(1);
      wPosition[6]->callback((Fl_Callback*)setPositionCB, (void*)(6));
      wPosition[6]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[6]
    { wPosition[7] = new Fl_Value_Slider(55, 610, 270, 20, "Y");
      wPosition[7]->type(1);
      wPosition[7]->labelsize(12);
      wPosition[7]->minimum(-250);
      wPosition[7]->maximum(250);
      wPosition[7]->step(1);
      wPosition[7]->callback((Fl_Callback*)setPositionCB, (void*)(7));
      wPosition[7]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[7]
    { wPosition[8] = new Fl_Value_Slider(55, 630, 270, 20, "Z");
      wPosition[8]->type(1);
      wPosition[8]->labelsize(12);
      wPosition[8]->minimum(-250);
      wPosition[8]->maximum(250);
      wPosition[8]->step(1);
      wPosition[8]->value(-157);
      wPosition[8]->callback((Fl_Callback*)setPositionCB, (void*)(8));
      wPosition[8]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[8]
    { wPosition[9] = new Fl_Value_Slider(365, 590, 270, 20, "X");
      wPosition[9]->type(1);
      wPosition[9]->labelsize(12);
      wPosition[9]->minimum(-250);
      wPosition[9]->maximum(250);
      wPosition[9]->step(1);
      wPosition[9]->callback((Fl_Callback*)setPositionCB, (void*)(9));
      wPosition[9]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[9]
    { wPosition[10] = new Fl_Value_Slider(365, 610, 270, 20, "Y");
      wPosition[10]->type(1);
      wPosition[10]->labelsize(12);
      wPosition[10]->minimum(-250);
      wPosition[10]->maximum(250);
      wPosition[10]->step(1);
      wPosition[10]->callback((Fl_Callback*)setPositionCB, (void*)(10));
      wPosition[10]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[10]
    { wPosition[11] = new Fl_Value_Slider(365, 630, 270, 20, "Z");
      wPosition[11]->type(1);
      wPosition[11]->labelsize(12);
      wPosition[11]->minimum(-250);
      wPosition[11]->maximum(250);
      wPosition[11]->step(1);
      wPosition[11]->value(-157);
      wPosition[11]->callback((Fl_Callback*)setPositionCB, (void*)(11));
      wPosition[11]->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wPosition[11]
    { wAllZ = new Fl_Value_Slider(365, 670, 270, 20, "Z");
      wAllZ->type(1);
      wAllZ->labelsize(12);
      wAllZ->minimum(-250);
      wAllZ->maximum(250);
      wAllZ->step(1);
      wAllZ->value(-157);
      wAllZ->callback((Fl_Callback*)setAllZCB);
      wAllZ->align(Fl_Align(FL_ALIGN_LEFT));
    } // Fl_Value_Slider* wAllZ
    o->end();
  } // Fl_Double_Window* o
  return w;
}