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


//Define impact convex hull
//For gear up
static const int ntdvtx_gearup = 13;
static TOUCHDOWNVTX tdvtx_gearup[ntdvtx_gearup] = {
	{_V(-0.0462, -1.3864, -1.8138), 1e7, 1e5, 3.0, 3.0},
	{_V(-15.1351, -0.1925, -7.3871), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0154, -1.5586, 3.1718), 1e7, 1e5, 3.0, 3.0},
	{_V(1.3849, -1.6702, 0.9560), 1e7, 1e5, 3.0, 3.0},
	{_V(-1.3849, -1.6702, 0.9560), 1e7, 1e5, 3.0, 3.0},
	{_V(14.9999, -0.1925, -7.3871), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0154, -0.5807, 6.1832), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0462, -0.6670, -5.8221), 1e7, 1e5, 3.0, 3.0},
	{_V(-1.7821, 0.3891, -5.8221), 1e7, 1e5, 3.0, 3.0},
	{_V(1.7690, 0.3891, -5.8221), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0154, 1.1046, 6.1832), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0154, 1.9840, 3.1718), 1e7, 1e5, 3.0, 3.0},
	{_V(-0.0154, 0.7840, -2.2576), 1e7, 1e5, 3.0, 3.0},
};
//For gear down
static const int ntdvtx_geardown = 3;
static TOUCHDOWNVTX tdvtx_geardown[ntdvtx_geardown] = {
	{_V(-0.0064, -2.5830, 5.6383), 1e6, 1e5, 1.6, 0.1},
	{_V(3.2436, -2.5830, -0.3017), 1e6, 1e5, 1.6, 0.1},
	{_V(-3.2864, -2.5830, -0.3017), 1e6, 1e5, 1.6, 0.1},
};

//HR1 class interface
class HR1: public VESSEL3{
    public:
        HR1 (OBJHANDLE hVessel, int flightmodel);
        ~HR1(); //Destructor
        void clbkSetClassCaps(FILEHANDLE cfg);
		int clbkConsumeBufferedKey(int, bool, char *);
		void SetGearUp();
		void SetGearDown();
	private:
	static void hlift(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
};

HR1::HR1(OBJHANDLE hVessel, int flightmodel)
:VESSEL3(hVessel, flightmodel){

}

HR1::~HR1(){

} //Destructor


// Overloaded callback functions
// Set the capabilities of the vessel class
void HR1::clbkSetClassCaps(FILEHANDLE cfg){
    THRUSTER_HANDLE th_main[2], th_rcs[15], th_group[4];
	THGROUP_HANDLE thg_main;

    //Physical vessel resources
    SetSize(HR1_SIZE);
    SetEmptyMass(HR1_EMPTYMASS);
	SetDockParams(HR1_DOCK_POS, HR1_DOCK_DIR, HR1_DOCK_ROT);
	SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
	CreateAirfoil3(LIFT_HORIZONTAL, HR1_CoP, hlift, NULL, HR1_HLIFT_C, HR1_HLIFT_S, HR1_HLIFT_A);

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
}

void HR1::SetGearUp(){
	HR1::SetTouchdownPoints(tdvtx_gearup, ntdvtx_gearup);
	//geardown = false;
}

void HR1::SetGearDown(){
	HR1::SetTouchdownPoints(tdvtx_geardown, ntdvtx_geardown);
	//geardown = true;
}

int HR1::clbkConsumeBufferedKey(int key, bool down, char *kstate){
    static bool gearToggled = false; // Variable para rastrear el cambio de tren de aterrizaje

    if (key == OAPI_KEY_G && down) {
        if (gearToggled) {
            SetGearUp();
        } else {
            SetGearDown();
        }
        gearToggled = !gearToggled; // Cambia el estado de la variable
    }

    return 0;
}

//Airfoil lift function
void HR1::hlift(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd){
	static const double clp[] = {  // lift coefficient from -pi to pi in 45deg steps
		0,0.4,0,-0.4,0,0.4,0,-0.4,0,0.4
	};
	static const double aoa_step = 45.0*RAD;
	double a, fidx;
	a = modf((aoa+PI)/aoa_step, &fidx);
	int idx = (int)(fidx+0.5);
	*cl = clp[idx]*(1.0-a) + clp[idx+1]*a;     // linear interpolation
	*cm = 0.0;
	*cd = 0.03;
	*cd += oapiGetInducedDrag (*cl, 1.5, 0.6); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}

//Vessel initialization
DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){
    return new HR1(hvessel, flightmodel);
}

//Vessel memory cleanup
DLLCLBK void ovcExit(VESSEL *vessel){
    if(vessel) delete(HR1*)vessel;
}