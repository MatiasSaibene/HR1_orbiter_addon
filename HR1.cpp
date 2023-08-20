// Copyright (c) Matías Saibene
// Licensed under the MIT License

// ==============================================================
//                 ORBITER MODULE: HR1
//
// HR1.cpp
// Control module for HR1 vessel class
//==============================================================
#define ORBITER_MODULE
#include "HR1.h"
#include <algorithm>
#include <stdio.h>
#include <strings.h>

bool lg_st_down = true;


HR1::HR1(OBJHANDLE hVessel, int flightmodel)
:VESSEL3(hVessel, flightmodel){
	landing_gear_proc = 0.0;
	landing_gear_status = GEAR_DOWN;
	DefineAnimations();
	hr1_vc = oapiLoadMeshGlobal("HR1_VC");
}

HR1::~HR1(){
} //Destructor

//Define animations
void HR1::DefineAnimations(void){
	//Close landing gear door
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

	anim_landing_gear = CreateAnimation(0.0);

	AddAnimationComponent(anim_landing_gear, 0, 1, &FrontLandingGearDoor_Rotate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &LandingGear_Rotate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Scale);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Translate);
	AddAnimationComponent(anim_landing_gear, 0, 1, &RearLandingGear_Close);
		
}

// Overloaded callback functions
// Set the capabilities of the vessel class
void HR1::clbkSetClassCaps(FILEHANDLE cfg){
    THRUSTER_HANDLE th_main[2], th_rcs[15], th_group[4];
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

    AddExhaust(th_main[0], 8, 1, _V(-1.0026, 0.0113, -6.0037), _V(0, 0, 1));
    AddExhaust(th_main[1], 8, 1, _V(0.9974, 0.0113, -6.0037), _V(0, 0, 1));

    //RCS Engines
	//Define RCS engines and locations
	th_rcs[ 0] = CreateThruster (_V( 1,0, 3), _V(0, 1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 1] = CreateThruster (_V( 1,0, 3), _V(0,-1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 2] = CreateThruster (_V(-1,0, 3), _V(0, 1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 3] = CreateThruster (_V(-1,0, 3), _V(0,-1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 4] = CreateThruster (_V( 1,0,-3), _V(0, 1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 5] = CreateThruster (_V( 1,0,-3), _V(0,-1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 6] = CreateThruster (_V(-1,0,-3), _V(0, 1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 7] = CreateThruster (_V(-1,0,-3), _V(0,-1,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 8] = CreateThruster (_V( 1,0, 3), _V(-1,0,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[ 9] = CreateThruster (_V(-1,0, 3), _V( 1,0,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[10] = CreateThruster (_V( 1,0,-3), _V(-1,0,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[11] = CreateThruster (_V(-1,0,-3), _V( 1,0,0), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[12] = CreateThruster (_V( 0,0,-3), _V(0,0, 1), HR1_MAXRCSTH, UXPS, HR1_ISP);
	th_rcs[13] = CreateThruster (_V( 0,0, 3), _V(0,0,-1), HR1_MAXRCSTH, UXPS, HR1_ISP);

	//Define RCS groups
	th_group[0] = th_rcs[0];
	th_group[1] = th_rcs[2];
	th_group[2] = th_rcs[5];
	th_group[3] = th_rcs[7];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_PITCHUP);

	th_group[0] = th_rcs[1];
	th_group[1] = th_rcs[3];
	th_group[2] = th_rcs[4];
	th_group[3] = th_rcs[6];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_PITCHDOWN);

	th_group[0] = th_rcs[0];
	th_group[1] = th_rcs[4];
	th_group[2] = th_rcs[3];
	th_group[3] = th_rcs[7];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_BANKLEFT);

	th_group[0] = th_rcs[1];
	th_group[1] = th_rcs[5];
	th_group[2] = th_rcs[2];
	th_group[3] = th_rcs[6];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_BANKRIGHT);

	th_group[0] = th_rcs[0];
	th_group[1] = th_rcs[4];
	th_group[2] = th_rcs[2];
	th_group[3] = th_rcs[6];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_UP);

	th_group[0] = th_rcs[1];
	th_group[1] = th_rcs[5];
	th_group[2] = th_rcs[3];
	th_group[3] = th_rcs[7];
	CreateThrusterGroup (th_group, 4, THGROUP_ATT_DOWN);

	th_group[0] = th_rcs[8];
	th_group[1] = th_rcs[11];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWLEFT);

	th_group[0] = th_rcs[9];
	th_group[1] = th_rcs[10];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_YAWRIGHT);

	th_group[0] = th_rcs[8];
	th_group[1] = th_rcs[10];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_LEFT);

	th_group[0] = th_rcs[9];
	th_group[1] = th_rcs[11];
	CreateThrusterGroup (th_group, 2, THGROUP_ATT_RIGHT);

	CreateThrusterGroup (th_rcs+12, 1, THGROUP_ATT_FORWARD);
	CreateThrusterGroup (th_rcs+13, 1, THGROUP_ATT_BACK);

    //Mesh for the visual
    AddMesh("HR1");

	mesh_Cockpit = AddMesh(hr1_vc, &HR1_COCKPIT_OFFSET);
	SetMeshVisibilityMode(mesh_Cockpit, MESHVIS_VC);
}

void HR1::clbkLoadStateEx(FILEHANDLE scn, void *vs){
	char *line;

	while(oapiReadScenario_nextline(scn, line)){
		if(!strncasecmp(line, "GEAR", 4)){
			sscanf(line+4, "%d%lf", (int *)&landing_gear_status, &landing_gear_proc);
		} else {
			ParseScenarioLineEx(line, vs);
		}
	}
	SetAnimation(anim_landing_gear, landing_gear_proc);
}

void HR1::clbkSaveState(FILEHANDLE scn){
	char cbuf[256];
	SaveDefaultState(scn);
	sprintf(cbuf, "%d %0.4f", landing_gear_status, landing_gear_proc);
	oapiWriteScenario_string(scn, "GEAR", cbuf);
}

void HR1::SetGearDown(void){
	ActivateLandingGear((landing_gear_status == GEAR_DOWN || landing_gear_status == GEAR_DEPLOYING) ?
		GEAR_STOWING : GEAR_DEPLOYING);
}

void HR1::ActivateLandingGear(LandingGearStatus action){
	landing_gear_status = action;
}

//Giving life to animations
void HR1::clbkPostStep(double simt, double simdt, double mjd){

	if(landing_gear_status >= GEAR_DEPLOYING){
		double da = simdt * LANDING_GEAR_OPERATING_SPEED;
		if(landing_gear_status == GEAR_DEPLOYING){
			if(landing_gear_proc > 0.0) landing_gear_proc = std::max(0.0, landing_gear_proc-da);
			else landing_gear_status = GEAR_DOWN;
			SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
		} else {
			if(landing_gear_proc < 1.0) landing_gear_proc = std::min(1.0, landing_gear_proc+da);
			else landing_gear_status = GEAR_UP;
			SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
		}
		SetAnimation(anim_landing_gear, landing_gear_proc);
	}
}

int HR1::clbkConsumeBufferedKey(int key, bool down, char *kstate){
	if(key == OAPI_KEY_G && down){
		SetGearDown();
		return 1;
	}
	return 0;
}

bool HR1::clbkLoadVC(int id){
	switch(id){
		case 0 : SetCameraOffset(_V(0.0015, 1.3051, 4.5500));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraRotationRange(RAD*120, RAD*120, RAD*60, RAD*60);
		break;//Commander's position	
	}
	return true;
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


//Vessel initialization
DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    return new HR1(hvessel, flightmodel);
}

//Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    if(vessel) delete(HR1*)vessel;
}