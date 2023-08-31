// Copyright (c) Matías Saibene
// Licensed under the MIT License

// ==============================================================
//                 ORBITER MODULE: HR1
//
// HR1.cpp
// Control module for HR1 vessel class
//==============================================================


#define ORBITER_MODULE
#include <cstdint>
#include "HR1.h"
#include <algorithm>
#include <stdio.h>
#include <strings.h>


//Constructor
HR1::HR1(OBJHANDLE hVessel, int flightmodel)
:VESSEL3(hVessel, flightmodel){
	landing_gear_proc = 0.0;
	landing_gear_status = GEAR_DOWN;
	docking_port_status = DCK_CLOSED;
	DefineAnimations();
	hr1_vc = oapiLoadMeshGlobal("HR1_VC");
	hPanelMesh = NULL;
}


HR1::~HR1(){
  if (hPanelMesh) oapiDeleteMesh (hPanelMesh);
} //Destructor

//Define animations
void HR1::DefineAnimations(void){
	//Landing gear
	static unsigned int FrontLandingGearDoorGrp[1] = {6};
	static MGROUP_ROTATE FrontLandingGearDoor_Rotate(
		0,
		FrontLandingGearDoorGrp,
		1,
		_V(-0.0119, -0.5923, 6.0787),
		_V(1, 0, 0),
		(float)(1.67552)
	);

	static unsigned int LandingGearRotateGrp[2] = {0, 1};
	static MGROUP_ROTATE LandingGear_Rotate(
		0,
		LandingGearRotateGrp,
		2,
		_V(0.0131, -0.3919, 5.6599),
		_V(1, 0, 0),
		(float)(1.57079)
	);

	static unsigned int RearLandingGear_ScaleGrp[1] = {3};
	static MGROUP_SCALE RearLandingGear_Scale(
		0,
		RearLandingGear_ScaleGrp,
		1,
		_V(0.0131, -0.3919, -0.2792),
		_V(1, 0.12, 1)
	);

	static unsigned int RearLandingGear_TranslateGrp[2] = {2, 4};
	static MGROUP_TRANSLATE RearLandingGear_Translate(
		0,
		RearLandingGear_TranslateGrp,
		2,
		_V(0, 2 , 0)
	);

	static unsigned int RearLandingGear_CloseGrp[1] = {7};
	static MGROUP_TRANSLATE RearLandingGear_Close(
		0,
		RearLandingGear_CloseGrp,
		1,
		_V(0, 0, -1.5)
	);

	//Docking port
	static unsigned int Docking_port_right_doorGrp[1] = {12};
	static MGROUP_ROTATE Docking_port_right_door(
		0,
		Docking_port_right_doorGrp,
		1,
		_V(0.5919, 1.8099, 1.7709),
		_V(0, 0, 1),
		(float)(-2.61799)
	);

	static unsigned int Docking_port_left_doorGrp[1] = {13};
	static MGROUP_ROTATE Docking_port_left_door(
		0,
		Docking_port_left_doorGrp,
		1,
		_V(-0.5934, 1.8099, 1.7709),
		_V(0, 0, 1),
		(float)(2.61799)
	);

	static unsigned int DockingPortGrp[1] = {11};
	static MGROUP_TRANSLATE DockingPort(
		0,
		DockingPortGrp,
		1,
		_V(0, 0.14, 0)
	);

	anim_landing_gear = CreateAnimation(0.0);

	AddAnimationComponent(anim_landing_gear, 0, 1, &FrontLandingGearDoor_Rotate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &LandingGear_Rotate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Scale);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Translate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Close);

	anim_docking_port = CreateAnimation(0.0);
	AddAnimationComponent(anim_docking_port, 0, 1, &Docking_port_right_door);
	AddAnimationComponent(anim_docking_port, 0, 1, &Docking_port_left_door);
	AddAnimationComponent(anim_docking_port, 0, 1, &DockingPort);
}

// Overloaded callback functions
// Set the capabilities of the vessel class
void HR1::clbkSetClassCaps(FILEHANDLE cfg){
    THRUSTER_HANDLE th_main[2], th_rcs[16], th_group[4];
	THGROUP_HANDLE thg_main;

	hwing = CreateAirfoil3 (LIFT_VERTICAL, _V(0,0,-0.3), vlift, 0, 5, 90, 1.5);
	// wing and body lift+drag components

	CreateAirfoil3 (LIFT_HORIZONTAL, _V(0,0,-4), hlift, 0, 5, 15, 1.5);

    //Physical vessel resources
    SetSize(HR1_SIZE);
    SetEmptyMass(HR1_EMPTYMASS);
	SetDockParams(HR1_DOCK_POS, HR1_DOCK_DIR, HR1_DOCK_ROT);

    //Propellant resources
    PROPELLANT_HANDLE UXPS = CreatePropellantResource(HR1_FUELMASS);

    //Main engine
	th_main[0] = CreateThruster(_V(-1.0026, 0.0113, -6.0037), _V(0, 0, 1), HR1_MAXMAINTH, UXPS, HR1_ISP);
	th_main[1] = CreateThruster(_V(0.9974, 0.0113, -6.0037), _V(0, 0, 1), HR1_MAXMAINTH, UXPS, HR1_ISP);
	thg_main = CreateThrusterGroup(th_main, 2, THGROUP_MAIN);
	

	SURFHANDLE exhaust_tex = oapiRegisterExhaustTexture("Exhaust");
    AddExhaust(th_main[0], 15, 1, _V(-1.0026, 0.0113, -6.0037), _V(0, 0, -1), exhaust_tex);
    AddExhaust(th_main[1], 15, 1, _V(0.9974, 0.0113, -6.0037), _V(0, 0, -1), exhaust_tex);

    //RCS Engines
	//Define RCS engines and locations and exhaust flames
	th_rcs[0] = CreateThruster (_V(-0.7582, 0.4888, 6.1890), _V(0, -1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[0], 1, 0.25, _V(-0.7582, 0.4888, 6.1890), _V(0, 1, 0), exhaust_tex);

	th_rcs[1] = CreateThruster(_V(10.4318, 0.4998, -3.6913), _V(0, -1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[1], 1, 0.25, _V(10.4318, 0.4998, -3.6913), _V(0, 1, 0), exhaust_tex);

	th_rcs[2] = CreateThruster(_V(-10.4462, 0.4998, -3.6913), _V(0, -1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[2], 1, 0.25, _V(-10.4462, 0.4998, -3.6913), _V(0, 1, 0), exhaust_tex);

	th_rcs[3] = CreateThruster(_V(10.4318, -0.5083, -3.6913), _V(0, 1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[3], 1, 0.25, _V(10.4318, -0.5083, -3.6913), _V(0, -1, 0), exhaust_tex);

	th_rcs[4] = CreateThruster(_V(-10.4462, -0.5104, -3.6913), _V(0, 1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[4], 1, 0.25, _V(-10.4462, -0.5104, -3.6913), _V(0, -1, 0), exhaust_tex);

	th_rcs[5] = CreateThruster(_V(-4.4950, -0.0364, 5.2247), _V(0, 0, -1), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[5], 1, 0.25, _V(-4.4950, -0.0364, 5.2247), _V(0, 0, 1), exhaust_tex);

	th_rcs[6] = CreateThruster(_V(4.5216, -0.0364, 5.2247), _V(0, 0, -1), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[6], 1, 0.25, _V(4.5216, -0.0364, 5.2247), _V(0, 0, 1), exhaust_tex);

	th_rcs[7] = CreateThruster(_V(2.0710, -0.0358, -5.1114), _V(-1, 0, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[7], 1, 0.25, _V(2.0710, -0.0358, -5.1114), _V(1, 0, 0), exhaust_tex);

	th_rcs[8] = CreateThruster(_V(-2.0725, -0.0358, -5.1114), _V(1, 0, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[8], 1, 0.25, _V(-2.0725, -0.0358, -5.1114), _V(-1, 0, 0), exhaust_tex);

	th_rcs[9] = CreateThruster(_V(2.8219, -0.0449, -2.5401), _V(0, 0, 1), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[9], 1, 0.25, _V(2.8219, -0.0449, -2.5401), _V(0, 0, -1), exhaust_tex);

	th_rcs[10] = CreateThruster(_V(-2.8372, -0.0449, -2.5401), _V(0, 0, 1), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[10], 1, 0.25, _V(-2.8372, -0.0449, -2.5401), _V(0, 0, -1), exhaust_tex);

	th_rcs[11] = CreateThruster(_V(3.1462, 0.6342, 2.3820), _V(-1, 0, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[11], 1, 0.25, _V(3.1462, 0.6342, 2.3820), _V(1, 0, 0), exhaust_tex);

	th_rcs[12] = CreateThruster(_V(-3.1389, 0.6342, 2.3820), _V(1, 0, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[12], 1, 0.25, _V(-3.1389, 0.6342, 2.3820), _V(-1, 0, 0), exhaust_tex);

	th_rcs[13] = CreateThruster(_V(0.7460, 0.4888, 6.1890), _V(0, -1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[13], 1, 0.25, _V(0.7460, 0.4888, 6.1890), _V(0, 1, 0));

	th_rcs[14] = CreateThruster(_V(0.7460, -0.3735, 6.1890), _V(0, 1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[14], 1, 0.25, _V(0.7460, -0.3735, 6.1890), _V(0, -1, 0));

	th_rcs[15] = CreateThruster(_V(-0.7582, -0.3715, 6.1890), _V(0, 1, 0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	AddExhaust(th_rcs[15], 1, 0.25, _V(-0.7582, -0.3715, 6.1890), _V(0, -1, 0));


	//Define RCS groups
	th_group[0] = th_rcs[1];
	th_group[1] = th_rcs[2];
	th_group[2] = th_rcs[14];
	th_group[3] = th_rcs[15];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_PITCHUP);

	th_group[0] = th_rcs[0];
	th_group[1] = th_rcs[13];
	th_group[2] = th_rcs[3];
	th_group[3] = th_rcs[4];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_PITCHDOWN);

	th_group[0] = th_rcs[2];
	th_group[1] = th_rcs[3];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_BANKLEFT);

	th_group[0] = th_rcs[1];
	th_group[1] = th_rcs[4];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_BANKRIGHT);


	th_group[0] = th_rcs[4];
	th_group[1] = th_rcs[3];
	th_group[2] = th_rcs[14];
	th_group[3] = th_rcs[15];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_UP);

	th_group[0] = th_rcs[0];
	th_group[1] = th_rcs[1];
	th_group[2] = th_rcs[2];
	th_group[2] = th_rcs[13];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_DOWN);

	th_group[0] = th_rcs[8];
	th_group[1] = th_rcs[11];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWLEFT);

	th_group[0] = th_rcs[7];
	th_group[1] = th_rcs[12];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWRIGHT);

	th_group[0] = th_rcs[7];
	th_group[1] = th_rcs[11];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_LEFT);

	th_group[0] = th_rcs[8];
	th_group[1] = th_rcs[12];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_RIGHT);

	th_group[0] = th_rcs[9];
	th_group[1] = th_rcs[10];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_FORWARD);

	th_group[0] = th_rcs[5];
	th_group[1] = th_rcs[6];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_BACK);

    //Mesh for the visual
    AddMesh("HR1");

	//Add a mesh for the virtual cockpit
	mesh_Cockpit = AddMesh(hr1_vc, &HR1_COCKPIT_OFFSET);
	SetMeshVisibilityMode(mesh_Cockpit, MESHVIS_VC);

	//Create control surfaces (code from DeltaGlider)
	CreateControlSurface3 (AIRCTRL_ELEVATOR, 1.4, 1.7, _V(0,0,-7.2), AIRCTRL_AXIS_XPOS, 1.0);
	CreateControlSurface3 (AIRCTRL_RUDDER, 0.8, 1.7, _V(0,0,-7.2), AIRCTRL_AXIS_YPOS, 1.0);
	hlaileron = CreateControlSurface3 (AIRCTRL_AILERON, 0.3, 1.7, _V( 7.5,0,-7.2), AIRCTRL_AXIS_XPOS, 1.0);
	hraileron = CreateControlSurface3 (AIRCTRL_AILERON, 0.3, 1.7, _V(-7.5,0,-7.2), AIRCTRL_AXIS_XNEG, 1.0);
	CreateControlSurface3 (AIRCTRL_ELEVATORTRIM, 0.3, 1.7, _V(   0,0,-7.2), AIRCTRL_AXIS_XPOS, 1.0);

}

//Load landing gear and docking port status from scenario file
void HR1::clbkLoadStateEx(FILEHANDLE scn, void *vs){
	char *line;

	while(oapiReadScenario_nextline(scn, line)){
		if(!strncasecmp(line, "GEAR", 4)){
			sscanf(line+4, "%d%lf", (int *)&landing_gear_status, &landing_gear_proc);
			SetAnimation(anim_landing_gear, landing_gear_proc);
		} else if(!strncasecmp(line, "DOCK", 4)){
			sscanf(line+4, "%d%lf", (int *)&docking_port_status, &docking_port_proc);
			SetAnimation(anim_docking_port, docking_port_proc);
		} else {
			ParseScenarioLineEx(line, vs);
		}
	}
}

//Save landing gear status to scenario file 
void HR1::clbkSaveState(FILEHANDLE scn){
	char cbuf[256];
	SaveDefaultState(scn);
	sprintf(cbuf, "%d %0.4f", landing_gear_status, landing_gear_proc);
	oapiWriteScenario_string(scn, "GEAR", cbuf);
	sprintf(cbuf, "%d %0.4f", docking_port_status, docking_port_proc);
	oapiWriteScenario_string(scn, "DOCK", cbuf);
}

void HR1::SetGearDown(void){
	ActivateLandingGear((landing_gear_status == GEAR_DOWN || landing_gear_status == GEAR_DEPLOYING) ?
		GEAR_STOWING : GEAR_DEPLOYING);
}

void HR1::CloseDockingPort(void){
	ActivateDockingPort((docking_port_status == DCK_CLOSED || docking_port_status == DCK_CLOSING) ?
		DCK_OPENING : DCK_CLOSING);
}

void HR1::ActivateLandingGear(LandingGearStatus action){
	landing_gear_status = action;
}

void HR1::ActivateDockingPort(DockingPortStatus actiondckp){
	docking_port_status = actiondckp;
}

//Giving life to animations
void HR1::clbkPostStep(double simt, double simdt, double mjd){
	UpdateLandingGearAnimation(simdt);
	UpdateDockingPortAnimation(simdt);
	SndBarrierEffect(simt);
}

void HR1::UpdateLandingGearAnimation(double simdt) {
    if (landing_gear_status >= GEAR_DEPLOYING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (landing_gear_status == GEAR_DEPLOYING) {
            if (landing_gear_proc > 0.0) landing_gear_proc = std::max(0.0, landing_gear_proc - da);
            else landing_gear_status = GEAR_DOWN;
            SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
        } else {
            if (landing_gear_proc < 1.0) landing_gear_proc = std::min(1.0, landing_gear_proc + da);
            else landing_gear_status = GEAR_UP;
            SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
        }
        SetAnimation(anim_landing_gear, landing_gear_proc);
    }
}

void HR1::UpdateDockingPortAnimation(double simdt) {
    if (docking_port_status >= DCK_CLOSING) {
        double da = simdt * LANDING_GEAR_OPERATING_SPEED;
        if (docking_port_status == DCK_CLOSING) {
            if (docking_port_proc > 0.0) docking_port_proc = std::max(0.0, docking_port_proc - da);
            else docking_port_status = DCK_CLOSED;
        } else {
            if (docking_port_proc < 1.0) docking_port_proc = std::min(1.0, docking_port_proc + da);
            else docking_port_status = DCK_OPEN;
        }
        SetAnimation(anim_docking_port, docking_port_proc);
    }
}

void HR1::SndBarrierEffect(double simt){
	//double machnumber = GetMachNumber();
	double airspeed = GetAirspeed();

	if((airspeed >= 340) && (airspeed <= 350)){
		//Sound speed barrier visual effect
		static PARTICLESTREAMSPEC soundbarrierpart = {
		0, 5.0, 16, 200, 0.15, 1.0, 5, 3.0, PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0, 2,
		PARTICLESTREAMSPEC::ATM_PLOG, 1e-4, 1};
		static VECTOR3 pos = {0, 2, 4};
		static VECTOR3 dir = {0, 1, 0};
		static double lvl = 0.1;
		AddParticleStream(&soundbarrierpart, pos, dir, &lvl);
	} else {
		DelExhaustStream(&soundbarrierpart);
	}
}

int HR1::clbkConsumeBufferedKey(int key, bool down, char *kstate){
	if(key == OAPI_KEY_G && down){
		SetGearDown();
		return 1;
	}
	if(key == OAPI_KEY_K && down){
		CloseDockingPort();
		return 1;
	}
	return 0;
}

//Configure virtual cockpit
bool HR1::clbkLoadVC(int id){
	switch(id){
		case 0 : SetCameraOffset(_V(0.0015, 1.3051, 4.5500));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraRotationRange(RAD*120, RAD*120, RAD*60, RAD*60);
		break;//Commander's position	
	}
	return true;
}

SURFHANDLE HR1::panel2dtex = NULL;

bool HR1::clbkLoadPanel2D (int id, PANELHANDLE hPanel,
  int viewW, int viewH)
{
  switch (id) {
  case 0 : 
    DefineMainPanel (hPanel);
    ScalePanel (hPanel, viewW, viewH);
    return true;
  default:
    return false;
  }
}



void HR1::DefineMainPanel (PANELHANDLE hPanel)
{
  static int panelW = 2048;
  static int panelH =  512;
  float fpanelW = (float)panelW;
  float fpanelH = (float)panelH;
  static int texW   = 2048;
  static int texH   =  512;
  float ftexW   = (float)texW;
  float ftexH   = (float)texH;
  static NTVERTEX VTX[4] = {
    {      0,      0,0,   0,0,0,            0.0f,1.0f-fpanelH/ftexH},
    {      0,fpanelH,0,   0,0,0,            0.0f,1.0f              },
    {fpanelW,fpanelH,0,   0,0,0,   fpanelW/ftexW,1.0f              },
    {fpanelW,      0,0,   0,0,0,   fpanelW/ftexW,1.0f-fpanelH/ftexH}
  };
  static uint16_t IDX[6] = {
    0,2,1,
    2,0,3
  };

  if (hPanelMesh) oapiDeleteMesh (hPanelMesh);
  hPanelMesh = oapiCreateMesh (0,0);
  MESHGROUP grp = {VTX, IDX, 4, 6, 0, 0, 0, 0, 0};
  oapiAddMeshGroup (hPanelMesh, &grp);
  SetPanelBackground (hPanel, &panel2dtex, 1, hPanelMesh, panelW, panelH, 0,
    PANEL_ATTACH_BOTTOM | PANEL_MOVEOUT_BOTTOM);

  static NTVERTEX VTX_MFD[4] = {
	{100, 50,0,    0,0,0,    0,0},
	{400, 50,0,    0,0,0,    1,0},
	{100,350,0,    0,0,0,    0,1},
	{400,350,0,    0,0,0,    1,1}
  };
  static uint16_t IDX_MFD[6] = {
	0,1,2,
	3,2,1
  };
  MESHGROUP grp_mfd = {VTX_MFD, IDX_MFD, 4, 6, 0, 0, 0, 0, 0};
  oapiAddMeshGroup(hPanelMesh, &grp_mfd);
  RegisterPanelMFDGeometry(hPanel, MFD_LEFT, 0, 1);
}

void HR1::ScalePanel (PANELHANDLE hPanel, int viewW, int viewH)
{
  double defscale = (double)viewW/1366.0;
  double magscale = std::max (defscale, 1.0);
  SetPanelScaling (hPanel, defscale, magscale);
}

//Airfoil lift function
void HR1::hlift(VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd){
	int i;
	const int nabsc = 8;
	static const double BETA[nabsc] = {-180*RAD,-135*RAD,-90*RAD,-45*RAD,45*RAD,90*RAD,135*RAD,180*RAD};
	static const double CL[nabsc]   = {       0,    +0.3,      0,   -0.3,  +0.3,     0,   -0.3,      0};
	for (i = 0; i < nabsc-1 && BETA[i+1] < beta; i++);
	if (i < nabsc - 1) {
		*cl = CL[i] + (CL[i + 1] - CL[i]) * (beta - BETA[i]) / (BETA[i + 1] - BETA[i]);
	}
	else {
		*cl = CL[nabsc - 1];
	}
	*cm = 0.0;
	*cd = 0.015 + oapiGetInducedDrag (*cl, 1.5, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}

void HR1::vlift(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;
	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {       0,      0,   -0.4,      0,    0.7,     1,   0.8,     0,      0};
	static const double CM[nabsc]  = {       0,      0,  0.014, 0.0039, -0.006,-0.008,-0.010,     0,      0};
	int i;
	for (i = 0; i < nabsc-1 && AOA[i+1] < aoa; i++);
	if (i < nabsc - 1) {
		double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
		*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
		*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	}
	else {
		*cl = CL[nabsc - 1];
		*cm = CM[nabsc - 1];
	}
	double saoa = sin(aoa);
	double pd = 0.015 + 0.4*saoa*saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag (*cl, 1.5, 0.7) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

DLLCLBK void InitModule (MODULEHANDLE hModule)
{

   HR1::panel2dtex = oapiLoadTexture ("HR1\\panel2d.dds");

}


DLLCLBK void ExitModule (MODULEHANDLE *hModule)
{
   
   oapiDestroySurface (HR1::panel2dtex);
    
}


//Vessel initialization
DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    HR1 *hr1vssl = new HR1(hvessel, flightmodel);
	return hr1vssl;
	//return new HR1(hvessel, flightmodel);
}


//Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    if(vessel) delete(HR1*)vessel;
}