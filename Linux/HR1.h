#ifndef __HR1_H
#define __HR1_H


#define STRICT 1
#include "OrbiterAPI.h"
#include "Orbitersdk.h"

//Vessel parameters
const double HR1_SIZE = 16; //Mean radius in meters.
const VECTOR3 HR1_CS = {29.83, 217.14, 30.92};
const double HR1_EMPTYMASS = 12000; //Empty mass in kg.
const double HR1_FUELMASS = 1000; //Fuel mass in kg.
const double HR1_ISP = 25e4; //Fuel-specific impulse in m/s.
//const double HR1_MAXMAINTH = 12e4; //Max main thrust.
const double HR1_MAXMAINTH = 2.0e5;
const double HR1_MAXHOVERTH = 1.5e4; //Max hover thrust.
//const double HR1_MAXRCSTH = 6e4; //Max RCS thrust.
const double HR1_MAXRCSTH = 8.0e3;
const VECTOR3 HR1_DOCK_POS = {-0.0563, 1.8099, 1.7193}; //Docking port location.
const VECTOR3 HR1_DOCK_DIR = {0, 1, 0}; //Docking port approach direction.
const VECTOR3 HR1_DOCK_ROT = {0, 0, -1}; //Docking port alignment direction.
const VECTOR3 HR1_CoP = {0, 0, 0}; //Center of pressure for airfoils in m.
const double HR1_HLIFT_C = 14.2; //Chord lenght in m.
const double HR1_HLIFT_S = 217.147; //Wing area in m^2.
const double HR1_HLIFT_A =  4.255; //Wing aspect ratio.
const double LANDING_GEAR_OPERATING_SPEED = 0.25;
const VECTOR3 HR1_COCKPIT_OFFSET = {0, 0, 0};

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
		enum LandingGearStatus {GEAR_DOWN, GEAR_UP, GEAR_DEPLOYING, GEAR_STOWING} landing_gear_status;
		enum DockingPortStatus {DCK_CLOSED, DCK_OPEN, DCK_CLOSING, DCK_OPENING} docking_port_status;
        HR1(OBJHANDLE hVessel, int flightmodel);
        virtual ~HR1(); //Destructor
		void DefineAnimations(void);
		void ActivateLandingGear(LandingGearStatus action);
		void SetGearDown(void);
		void ActivateDockingPort(DockingPortStatus actiondckp);
		void CloseDockingPort(void);
		void UpdateLandingGearAnimation(double);
		void UpdateDockingPortAnimation(double);
		void SndBarrierEffect(double);


        void clbkSetClassCaps(FILEHANDLE cfg) override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs) override;
        void clbkSaveState(FILEHANDLE scn) override;
		void clbkPostStep(double, double, double) override;
		int clbkConsumeBufferedKey(int, bool, char *) override;
		bool clbkLoadVC(int id) override;
		virtual bool clbkLoadPanel2D(int id, PANELHANDLE hPanel, int viewW, int viewH) override;
		static void hlift(VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd);
		static void vlift(VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
		void DefineMainPanel (PANELHANDLE hPanel);
		void ScalePanel (PANELHANDLE hPanel, int viewW, int viewH);	

		MESHHANDLE hr1_vc;
		MESHHANDLE hPanelMesh;
		unsigned int mesh_Cockpit;
		static SURFHANDLE panel2dtex;
		PARTICLESTREAMSPEC soundbarrierpart;
		PSTREAM_HANDLE sndbarrier_fx;

	private:
	unsigned int anim_landing_gear;
	unsigned int anim_docking_port;
	double landing_gear_proc;
	double docking_port_proc;
	AIRFOILHANDLE hwing;
	CTRLSURFHANDLE hlaileron, hraileron;
	
};

#endif  // !__HR1_H