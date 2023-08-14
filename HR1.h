#ifndef __HR1_H
#define __HR1_H

#define STRICT 1

#include "Orbitersdk.h"

//Vessel parameters
const double HR1_SIZE = 16; //Mean radius in meters.
const VECTOR3 HR1_CS = {29.83, 217.14, 30.92};
const double HR1_EMPTYMASS = 12000; //Empty mass in kg.
const double HR1_FUELMASS = 1000; //Fuel mass in kg.
const double HR1_ISP = 25e4; //Fuel-specific impulse in m/s.
const double HR1_MAXMAINTH = 12e4; //Max main thrust.
const double HR1_MAXHOVERTH = 1.5e4; //Max hover thrust.
const double HR1_MAXRCSTH = 6e4; //Max RCS thrust.
const VECTOR3 HR1_DOCK_POS = {-0.0563, 1.8099, 1.7193}; //Docking port location.
const VECTOR3 HR1_DOCK_DIR = {0, 1, 0}; //Docking port approach direction.
const VECTOR3 HR1_DOCK_ROT = {0, 0, -1}; //Docking port alignment direction.
const VECTOR3 HR1_CoP = {0, 0, 0}; //Center of pressure for airfoils in m.
const double HR1_HLIFT_C = 14.2; //Chord lenght in m.
const double HR1_HLIFT_S = 217.147; //Wing area in m^2.
const double HR1_HLIFT_A =  4.255; //Wing aspect ratio.
#endif