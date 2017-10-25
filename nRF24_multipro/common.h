#ifndef _COMMON_H
#define _COMMON_H

// supported protocols
enum {
	PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
	PROTO_CG023,        // EAchine CG023, CG032, 3D X4
	PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
	PROTO_CX10_GREEN,   // Cheerson CX-10 green board
	PROTO_H7,           // EAchine H7, MoonTop M99xx
	PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
	PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
	PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
	PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
	PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
	PROTO_SYMAXOLD,     // Syma X5C, X2
	PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
	PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
	PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
	PROTO_FQ777124,     // FQ777-124 pocket drone
	PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
	PROTO_BAYANG_SILVERWARE, // Bayang for Silverware with frsky telemetry
	PROTO_END
};

enum chan_order {
	THROTTLE,
	AILERON,
	ELEVATOR,
	RUDDER,
	AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
	AUX2,  // (CH6)  flip control
	AUX3,  // (CH7)  still camera (snapshot)
	AUX4,  // (CH8)  video camera
	AUX5,  // (CH9)  headless
	AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
	AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
	AUX8,  // (CH12) Reset / Rebind
};

struct telemetry_data_t {
	uint16_t volt1;
	uint16_t rssi;
	uint8_t updated;
	uint32_t lastUpdate;
};

extern struct telemetry_data_t telemetry_data;
extern uint16_t ppm[12];
extern uint8_t packet[32];
extern uint8_t transmitterID[4];
extern uint8_t current_protocol;
extern bool reset;

#endif
