//g++ -o serial serial.c -std=gnu++11 -L CAN/PCAN_Basic_Linux-2.0.3/pcanbasic/ -lpcanbasic -pthread -lrt
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <stdbool.h>
#include <signal.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <arpa/inet.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/time.h>
#include <asm/types.h>
#include <unistd.h>

#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
//#include "../pcanbasic/PCANBasic.h"
#include "/home/min/CAN/PCAN_Basic_Linux-2.0.3/pcanbasic/PCANBasic.h"
/************************************************************************************************************************************************/
#define dev_t       int

#define UBX_SYNCCHAR_ONE	0xB5
#define UBX_SYNCCHAR_TWO	0x62

#define PRINT_BUF_MAX_SIZE 	10240

typedef struct {		// serial control type
    dev_t dev;			// serial device
    int error;			// error state
} serial_t;

typedef struct __GNRMC{
   char ID[7] = "$GNRMC";	// Message ID: Recommended Location Information
   unsigned char hour;		// GPS hour in one day
   unsigned char miniute;	// GPS miniute in an hour
   double second;		// GPS second in one miniute
   char isVailed;		// A=vailed, V=invailed
   double latitude;		// Latitude in degree, ddmm.mmmm
   char southorNorth;		// N=Northern Hemisphere, S=Southern Hemisphere
   double longitude;		// Longitude in degree, dddmm.mmmm
   char eastorWest;             // E=Eastern Hemisphere, W=Western Hemisphere
   double speedk;		// ground rate in (000.0~999.9)knots
   double heading;		// ground course(000.0~359.9)degree
   int year;			// GPS year
   unsigned char month;		// GPS month in year
   unsigned char day;		// GPS day in month
   double declination;		// Declination (000.0~180.0)degree
   char direction;		// Direction E=east, W=west
   char mode;			// A=Autonomous positioning, D=Differential positioning, E=Estimate， N=Invaild
}tGNRMC;

typedef struct __GNGLL{
   char ID[7] = "$GNGLL";	// Message ID: Geographic Position
   double latitude;		// Latitude in degree, ddmm.mmmm
   char southorNorth;           // N=Northern Hemisphere, S=Southern Hemisphere
   double longitude;		// Longitude in degree, dddmm.mmmm
   char eastorWest;             // E=Eastern Hemisphere, W=Western Hemisphere
   unsigned char hour;		// GPS hour in one day
   unsigned char miniute;	// GPS miniute in an hour
   double second;		// GPS second in one miniute
   char isVailed;		// A=vailed, V=invailed
   char mode;			// A=Autonomous positioning, D=Differential positioning, E=Estimate， N=Invaild
}tGNGLL;


typedef struct __GNGGA{
   char ID[7] = "$GNGGA";	//Global Positioning System Fix Data
   unsigned char hour;		// GPS hour in one day
   unsigned char miniute;	// GPS miniute in an hour
   double second;		// GPS second in one miniute
   double latitude;		// Latitude in degree, ddmm.mmmm
   char southorNorth;		// N=Northern Hemisphere, S=Southern Hemisphere
   double longitude;		// Longitude in degree, dddmm.mmmm
   char eastorWest;             // E=Eastern Hemisphere, W=Western Hemisphere
   unsigned char status;	// 0=Fix Not Vailed, 1=GPS Fix, 2=DGPS 3=RTK Fix, 5=RTK Float, 6=Estmatting
   unsigned char numSatellite;	// the number of the using satellites(00~12)
   double hdop;			// Horizontal dilution of precision
   double altitude;		// Altitufde in meter, (-9999.9 - 99999.9)
   char altUnit;		// M=meter
   double geoid;		// Division of the high WGS84 level of the earth ellipsoid relative to the earth's surface
   char geoUnit;		// M=meter
   double diffSec;		// The difference of time (from a few seconds, receiving the differential signal the beginning if not differential positioning will be empty)
   unsigned short numBaseStation;       // number of the Base station (0000 - 1023)
}tGNGGA;

typedef struct __GNVTG{
   char ID[7] = "$GNVTG";	// Track Made Good and Ground Speed
   double rheading;		// Angle of motion
   char rhUnit;			// T=Real North reference system
   double mheading;             // Angle of motion
   char mhUnit;			// M=Magnetic North reference system
   double kspeed;		// Horizontal movement speed
   char ksUnit;                 // N=Knot
   double hspeed;		// Horizontal movement speed
   char hsUnit;			// K=km/h
   char mode;			// A=Autonomous positioning, D=Differential positioning, E=Estimate， N=Invaild
}tGNVTG;

typedef struct __GNGSA{
   char ID[7] = "$GNGSA";	// Message ID: GPS DOP and Active Satellites
   char mode; 			// A=auto 2D/3D; M=manual 2D/3D
   unsigned char type; 		// 1=not Fix, 2=2D, 3=3D
   unsigned char prn[12]; 	// The PRN code of the current used satellites on 1~12 channel
   double pdop;                 // Position dilution of precision
   double hdop;                 // Horizontal dilution of precision
   double vdop;                 // Vertical dilution of precision
}tGNGSA;

typedef struct __GXGSV{
   char ID[7] = "$GPGSV";	// Message ID: GPS Satellites in View
   unsigned char amount;	// GSV message amount(1-4)
   unsigned char num;		// Number of this GSV message(1-4)
   unsigned char numSatellite;  // Nmumber of the visiable satellite(0-16)
   struct __SatelliteInfo{
      unsigned char prn;	// Pseudo random noise code(1-32)
      unsigned char elevation;	// Satellite elevation(00 - 90)
      unsigned short azimuth;	// Satellite azimuth(00 - 359)
      unsigned char snr;	// Signal-to-noise ratio((00～99dB))
   }satellite[4];
}tGXGSV;

//CAN MESSAGE ID LIST
enum{
    TCU_268 = 0x268,
};

//CAN Data Describe for CHANGAN
union can_data{
    char payload[8];

   /// Transmission Control Unit Data
    struct __TCU_268{
        unsigned char AT_RESERVED_BIT0:7;           /*Reserved for Future, 7 Bits*/
        unsigned char AT_HeavyDeleclerationFlag:1;  /*Heavy Delecleration Flag
                                                      0x0: OFF
                                                      0X1: ON*/
        unsigned char AT_ActualGear:4;              /*Actual Gear
                                                      0x00: Neutral;
                                                      0x00: 1st gear
                                                      0x02: 2nd gear
                                                      0x03: 3rd gear
                                                      0x04: 4th gear
                                                      0x05: 5th gear
                                                      0x06: 6th gear
                                                      0x07: reserved for future
                                                      0x08: reserved for future
                                                      0x09: reserved for future
                                                      0x0A: Parking
                                                      0x0B: reserved for future
                                                      0x0C: reserved for future
                                                      0x0D: reserved for future
                                                      0x0E: reserved for future
                                                      0x0F: Invalid*/
        unsigned char AT_RESERVED_BIT1:4;           /*Reserved for Future, 4 Bites*/
        unsigned char AT_RESERVED_BYTE0;            /*Reserved for Future, 1 Byte*/
        unsigned char AT_VehicleSpeed_MSB:5;        /*Vehicle Speed MSB, 5 Bites
                                                      Resolution: 0.1
                                                      Initial value: 0
                                                      Invalid value: 0x1FFF
                                                      Offset: 0
                                                      Uint: km/h
                                                      Min: 0 (0x00)
                                                      Max: 360 (0x1900)*/
        unsigned char AT_RESERVED_BIT2:3;           /*Reserved for Future, 3 Bites*/
        unsigned char AT_VehicleSpeed_LSB;          /*Vehicle Speed LSB*/
        unsigned char AT_RESERVED_BYTE1[3];         /*Reserved for Future, 3 Bytes*/
    }tcu_268;

}__attribute__((packed));


//CAN Frame Detail
struct can_frame_ext
{
    unsigned int can_id:11;
    unsigned int can_id_ext:18;
    unsigned int err_flag:1;
    unsigned int rt_request:1;
    unsigned int extended:1;
    unsigned int can_dlc;
    union can_data data;
};

//CAN Receive Buffer
typedef union __CAN_FRAME
{
    struct can_frame frame;
    struct can_frame_ext frame_ext;
}rcv_buff_t;

// UBX ESF MEAS
typedef struct __ESF_MEAS_FLAGS{
   unsigned short timeMarkSent:2;
   unsigned short timeMarkEdge:1;
   unsigned short calibTtagValid:1;
   unsigned short reserved:4;
}__attribute__((packed)) t_esf_meas_flags;

typedef struct __ESF_MEAS_BITFIELD{
   unsigned int dataField:24;
   unsigned int dataType:6;
   unsigned int reserved:2;
}__attribute__((packed)) t_esf_meas_data;

typedef struct __INPUT_ESF_MEAS{
   const unsigned short header = htons(0xb562);
   const unsigned char Class = 0x10;
   const unsigned char ID = 0x02;
   const unsigned short length = 20;
   unsigned int timeTag;
   t_esf_meas_flags flags;
   unsigned short id;
   t_esf_meas_data data;
   unsigned short crc;
}__attribute__((packed)) t_input_esf_meas; 

typedef struct __STR_INPUT_ESF_MEAS{
   const char* timeTag="Time tag of measurement generated by external sensor:";
   const char* timeMarkSent="Time mark signal was supplied just prior to sending this message(0:none, 1:on Ext0, 2:=on Ext1):";
   const char* timeMarkEdge="Trigger on rising(0) or falling(1) edge of time mark signal:";
   const char* calibTtagValid="Calibration time tag available. Always set to zero:";
   const char* id="Identification number of data provider:";
   const char* dataField="Data:";
   const char* dataType="Type of data(0 = no data; 1...63=data type):";
}__attribute__((packed))t_str_input_esf_meas;

// UBX NAV ATT
typedef struct __PERIODIC_POLLED_NAV_ATT{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x01;
   const unsigned char ID = 0x05;
   const unsigned short length = 32;
   unsigned int iTOW;
   unsigned char version;
   unsigned char reserved1[3];
   signed int roll;
   signed int pitch;
   signed int heading;
   signed int accRoll;
   signed int accPitch;
   signed int accHeading;
   unsigned short crc;
}t_periodic_polled_nav_att;

typedef struct __STR_NAV_ATT{
   const char* iTOW="GPS Time of week of the navigation epoch:";
   const char* version="Message version(0 for this version):";
   const char* roll="Vehicle roll<deg>";
   const char* pitch="Vehicle pitch<deg>";
   const char* heading="Vehicle heading<deg>";
   const char* accRoll="Vehicle roll accuracy(if null, roll angle is not available)<deg>";
   const char* accPitch="Vehicle pitch accuracy(if null, pitch angle is not available)<deg>";
   const char* accHeading="Vehicle heading accuracy(if null, heading angle is not available)<deg>";
}t_str_nav_att;

// UBX ESF INS
typedef struct __ESF_INS_BITFIELD0{
   unsigned int version:8;
   unsigned int xAngRateValid:1;
   unsigned int yAngRateValid:1;
   unsigned int zAngRateValid:1;
   unsigned int xAccelValid:1;
   unsigned int yAccelValid:1;
   unsigned int zAccelValid:1;
   unsigned int reserved:18;
}t_esf_ins_bitfield0;

typedef struct __PERIODIC_POLLED_ESF_INS_{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x10;
   const unsigned char ID = 0x15;
   const unsigned short length = 36;
   t_esf_ins_bitfield0 bitfield0;
   unsigned char reserved[4];
   unsigned int iTOW;
   signed int xAngRate;
   signed int yAngRate;
   signed int zAngRate;
   signed int xAccel;
   signed int yAccel;
   signed int zAccel;
   unsigned short crc;
}t_periodic_polled_esf_ins;

typedef struct __STR_ESF_INS{
   const char* version="Message version (1 for this version):";
   const char* xAngRateValid="Compensated x-axis angular rate data validity flag(0:not valid, 1: valid):";
   const char* yAngRateValid="Compensated y-axis angular rate data validity flag(0:not valid, 1: valid):";
   const char* zAngRateValid="Compensated z-axis angular rate data validity flag(0:not valid, 1: valid):";
   const char* xAccelValid="Compensated z-axis acceleration data validity flag(0:not valid, 1: valid):";
   const char* yAccelValid="Compensated y-axis acceleration data validity flag(0:not valid, 1: valid):";
   const char* zAccelValid="Compensated z-axis acceleration data validity flag(0:not valid, 1: valid):";
   const char* iTOW="GPS time of week of the navigation epoch:";
   const char* xAngRate="Compensated x-axis angular rate<deg/s>:";
   const char* yAngRate="Compensated y-axis angular rate<deg/s>:";
   const char* zAngRate="Compensated z-axis angular rate<deg/s>:";
   const char* xAccel="Compensated x-axis acceleration(gravity-free)<mg>:";
   const char* yAccel="Compensated y-axis acceleration(gravity-free)<mg>:";
   const char* zAccel="Compensated z-axis acceleration(gravity-free)<mg>:";
}t_str_esf_ins;

//UBX Sensor mount aligment information
typedef struct __ESF_ALG_BITFIELD0{
   unsigned char status:3;
   unsigned char error:1;
   unsigned char reserved:4;
}__attribute__((packed)) t_esf_alg_bitfield0;

typedef struct __PERIODIC_POLLED_ESF_ALG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x10;
   const unsigned char ID = 0x14;
   const unsigned short length = 16;
   unsigned int iTOW;
   unsigned char version;
   t_esf_alg_bitfield0 bitfield0;
   unsigned char reserved1[4];
   signed short roll;
   signed short pitch;
   signed short yaw;
   unsigned short crc;
}__attribute__((packed)) t_periodic_polled_esf_alg;

typedef struct __STR_ESF_ALG{
   const char* iTOW="GPS time of week of the navigation epoch:";
   const char* version="Message version:";
   const char* status_tag="status of the sensor mount alignment:";
   const char* status[4]={"user-defined","estimating sensor mount roll/pitch angles","estimating sensor mount roll/pitch/yaw angles","automatic sensor-mount alignment is completed"};
   const char* error="Automatic sensor mount alignment error:";
   const char* roll="Sensor mount roll misalignment angle<deg>:";
   const char* pitch="Sensor mount pitch misalignment angle<deg>:";
   const char* yaw="Sensor mount yaw misalignment angle<deg>:";
}__attribute__((packed)) t_str_esf_alg;

//UBX Sensor fusion status information(ESF STATUS)
typedef struct __ESF_STATUS_INITSTATUS{
   unsigned char wtInitStatus:2;
   unsigned char mntAlgStatus:3;
   unsigned char reserved:3;
}__attribute__((packed)) t_esf_status_initStatus;

typedef struct __ESF_STATUS_SENSSTATUS1{
   unsigned char type:6;
   unsigned char used:1;
   unsigned char ready:1;
}__attribute__((packed)) t_esf_status_sens1Status;

typedef struct __ESF_STATUS_SENSSTATUS2{
   unsigned char calibStatus:2;
   unsigned char timeStatus:2;
   unsigned char reserved:4;
}__attribute__((packed)) t_esf_status_sens2Status;

typedef struct __ESF_STATUS_FAULTS{
   unsigned char badMeas:1;
   unsigned char badTTag:1;
   unsigned char missingMeas:1;
   unsigned char noisyMeas:1;
   unsigned char reserved:4;
}__attribute__((packed)) t_esf_status_faults;

typedef struct __ESF_STATUS_SENS{
   t_esf_status_sens1Status sensStatus1;
   t_esf_status_sens2Status sensStatus2;
   unsigned char freq;
   t_esf_status_faults faults;
}__attribute__((packed)) t_esf_status_sens;

typedef struct __PERIODIC_POLLED_ESF_STATUS{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x10;
   const unsigned char ID = 0x10;
   const unsigned short length = 16 + 7*4;
   unsigned int iTOW;
   unsigned char version;
   t_esf_status_initStatus initStatus;
   unsigned char reserved1[6];
   unsigned char status;
   unsigned char reserved2[2];
   unsigned char numSens;
   t_esf_status_sens sens;
   unsigned short crc;
}__attribute__((packed)) t_periodic_polled_esf_status;

typedef struct __STR_ESF_STATUS{
   const char *iTOW="GPS time of week of the navigation epoch(iTow):";
   const char *version="Message Version(2 for this version):";
   const char *initStatus_wtInit="Wheel-tick factor initialization status:";
   const char *wtInitStatus[3]={"off", "initializing", "initialized"};
   const char *initStatus_mntAlg="Automatic sensor-mount alignment status:";
   const char *mntAlgStatus[3]={"off","initializing","initialized"};
   const char *status_tag="Sensor fuision status:";
   const char *status[4]={"no fuision","fuision, GNSS and sensor data are used","disable temporarily, invalid sensor data not used, e.g. car on ferry", "disable permanently until receiver reset, GNSS-only due to sensor failure"};
   const char *numSens="Number of sensors:";
   const char *sensStatus1_type="Sensor data type:";
   const char *sensStatus1_used="Sensor data was used for the current solution:";
   const char *sensStatus1_ready="Sensor configure is available or not required:";
   const char *sensStatus2_calib="Calibration status:";
   const char *calibStatus[4]={"No calibration", "Calibrating, sensor not yet calibrated", "Calibrating, sensor coarsely calibrated", "Calibrating, sensor finely calibrated"};
   const char *sensStatus2_time="Time tag status:";
   const char *timeStatus[4]={"No data", "Reception of the first byte used to tag the measurement", "Event input used to tag the measurement", "Time tag provided with the data"};
   const char *freq="Observation frequency:";
   const char *faults_badMeas="Bad measurements seen:";
   const char *faults_badTTag="Bad measurement ttags seen:";
   const char *faults_missingMeas="Measurements missing or misaligned:";
   const char *faults_noisyMeas="Measurement noise is high:";
}__attribute__((packed)) t_str_esf_status;

//UBX High Rate Output of PVT Solution (NAV HNR)
typedef struct __NAV_HNR_VALID{
   unsigned char validDate:1;
   unsigned char validTime:1;
   unsigned char fullyResolved:1;
   unsigned char reserved:5;
}__attribute__((packed)) t_nav_hnr_valid;

typedef struct __NAV_HNR_FLAGS{
   unsigned char GPSfixOK:1;
   unsigned char DiffSoln:1;
   unsigned char WKNSET:1;
   unsigned char TOWSET:1;
   unsigned char headVehValid:1;
   unsigned char reserved:3;
}__attribute__((packed)) t_nav_hnr_flags;

typedef struct __PERIODIC_POLLED_NAV_HNR{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x01;
   const unsigned char ID = 0x37;
   const unsigned short length = 52;
   unsigned int TOW;
   unsigned short year;
   unsigned char month;
   unsigned char day;
   unsigned char hour;
   unsigned char min;
   unsigned char sec;
   t_nav_hnr_valid valid;
   signed int nano;
   unsigned char gpsFix;
   t_nav_hnr_flags flags;
   unsigned char reserved1[2];
   signed int lon;
   signed int lat;
   signed int height;
   signed int hMSL;
   signed int gSpeed;
   signed int speed;
   signed int headMot;
   signed int headVeh;
   unsigned short crc;
}__attribute__((packed)) t_periodic_polled_nav_hnr;

typedef struct __STR_NAV_HNR{
   const char *TOW="GPS time of week of the navigation solution(TOW):";
   const char *valids[3]={"vaildDate","vaildTime","fullResolved"};
   const char *fixStatus[6]={"No Fix","Dead Reckoning only","2D-Fix","3D-Fix","GPS+dead reckoning combined","Time only Fix"};
   const char *flags[5]={"Heading of vehicle is vaild", "Valid GPS time of week(iTOW & fTOW)", "Valid GPS week number", "DGPS used", "Fix within limits(e.g. DOP & accuracy)"};
   const char *latitude="Longitude<deg>:";
   const char *longitude="Latitude<deg>:";
   const char *gspeed="Ground Speed(2-D)<m/s>:";
   const char *speed="Speed(3-D)<m/s>:";
   const char *height="Height above Ellipsoid<m>:";
   const char *hMSL="Height above mean sea level<m>:";
   const char *headMot="Heading of motion(2-D)<deg>:";
   const char *headVeh="Heading of vehicle(2-D)<deg>:";
}__attribute__((packed)) t_str_nav_hnr;

//UBX CFG HNR
typedef struct __STR_CFG_HNR{
   const char* highNavRate="Rate of navigation solution output:";
}__attribute__((packed))t_str_cfg_hnr;

typedef struct __POLL_CFG_HNR{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x5c;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_hnr;

typedef struct __SET_GET_CFG_HNR{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x5c;
   const unsigned short length = 0x04;
   unsigned char highNavRate;
   unsigned char reserved1[3];
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_hnr;


//UBX CFG RATE
typedef struct __POLL_CFG_RATE{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x08;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_rate;

typedef struct __SET_GET_CFG_RATE{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x08;
   const unsigned short length = 0x06;
   unsigned short measRate;
   unsigned short navRate;
   unsigned short timeRef;
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_rate;

typedef struct __STR_CFG_RATE{
   const char *measRate="Measurement Rate, GPS measurements are taken every measRate milliseconds:";
   const char *navRate="Navigation Rate in number of measurement cycles. (This parameter cannot be changed, and must be set to 1):";
   const char *timeRef="Alignment to reference time:";
}t_str_cfg_rate;

//UBX CFG NAVX5
typedef struct __CFG_MASK1{
   unsigned short reserved1:2;
   unsigned short minMax:1;
   unsigned short minCn0:1;
   unsigned short reserved2:2;
   unsigned short initial3dfix:1;
   unsigned short reserved3:2;
   unsigned short wknRoll:1;
   unsigned short ackAid:1;
   unsigned short reserved4:2;
   unsigned short ppp:1;
   unsigned short aop:1;
   unsigned short reserved5:1;
}t_cfg_navx5_mask1;

typedef struct __CFG_MASK2{
   unsigned int reserved1:6;
   unsigned int adr:1;
   unsigned int reserved2:25;
}t_cfg_navx5_mask2;

typedef struct __CFG_NAVX5_AOPCFG{
   unsigned char useAOP:1;
   unsigned char reserved:7;
}t_cfg_navx5_aopCfg;

typedef struct __SET_GET_CFG_NAVX5{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x23;
   const unsigned short length = 40;
   unsigned short version;
   t_cfg_navx5_mask1 mask1;
   t_cfg_navx5_mask2 mask2;
   unsigned char reserved1[2];
   unsigned char minSVs;
   unsigned char maxSVs;
   unsigned char minCN0;
   unsigned char reserved2;
   unsigned char iniFix3D;
   unsigned char reserved3[2];
   unsigned char ackAiding;
   unsigned short wknRollover;
   unsigned char reserved4[6];
   unsigned char usePPP;
   t_cfg_navx5_aopCfg aopCfg;
   unsigned char reserved5[3];
   unsigned char aopOrbMaxErr;
   unsigned char reserved6[4];
   unsigned char reserved7[3];
   unsigned char useAdr;
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_navx5;

typedef struct __POLL_CFG_NAVX5{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x23;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_navx5;

typedef struct __STR_CFG_NAVX5{
   const char* minMax="apply min/max SVs setting:";
   const char* minCn0="apply minimum C/N0 setting:";
   const char* initial3dfix="apply initial 3D fix settings:";
   const char* wknRoll="apply GPS weeknumber rollover settings:";
   const char* ackAid="apply assistance acknowledgement settings:";
   const char* ppp="apply usePPP flag:";
   const char* aop="apply aopCfg(useAOP flag) and aopOrbMaxErr settings(AssistNow Autonomous):";
   const char* adr="apply ADR usage setting(useAdr flag):";
   const char* minSVs="Minimum number of satellites for navigation:";
   const char* maxSVs="Maximum number of satellites for navigation:";
   const char* minCN0="Minimum satellite signal level for navigation:";
   const char* iniFix3D="1=initial fix must be 3D:";
   const char* ackAiding="1=issue acknowldgements for assistance message input:";
   const char* wknRollover="GPS week rollover number:";
   const char* usePPP="1=use Precise Point Positioning:";
   const char* aopCfg="AssistNow Autonomous configuration:";
   const char* aopOrbMaxErr="Maximum acceptable(modeled) AssistNow Autonomous orbit error(valid range=5..1000, or 0=reset to firmware default)<m>:";
   const char* useAdr="Enable/disable ADR(0:ADR is disabled, 1:ADR is enabled):";
}__attribute__((packed)) t_str_cfg_navx5;

//UBX CFG ESFALG
typedef struct __CFG_ESFALG_BITFIELD{
   unsigned int version:8;
   unsigned int doAutoMntAlg:1;
   unsigned int reserved:23;
}__attribute__((packed)) t_cfg_esfalg_bitfield;

typedef struct __POLL_CFG_ESFALG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x56;
   const unsigned short length = 12;
   t_cfg_esfalg_bitfield bitfield; // bit8: doAutoMnuAlg 0:Disabled; 1:Enabled  bit7: version(0 for this version)
   unsigned int yaw; 	// 0~360
   signed short pitch; 	// -90~90
   signed short roll;	// -180~180
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_esfalg;

typedef struct __SET_GET_CFG_ESFALG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x56;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_esfalg;

typedef struct __STR_CFG_ESFALG{
   const char* version="Message version(0 for this version):";
   const char* doAutoMntAlg="Enable/disable automatic sensor mount alignment(0:Disabled, 1:Enabled):";
   const char* yaw="Sensor mount yaw angle[0,360]<deg>:";
   const char* pitch="Sensor mount pitch angle[-90,90]<deg>:";
   const char* roll="Sensor mount roll angle[-180,180]<deg>:";
}__attribute__((packed)) t_str_cfg_esfalg;

//UBX CFG ESFLA
typedef struct __POLL_CFG_ESFLA{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x2f;
   const unsigned short length = 4+8;
   unsigned char version;
   unsigned char numConfigs;
   unsigned char reserved1[2];
   unsigned char sensType0;
   unsigned char reserved2_0;
   unsigned short leverArmX0; // cm
   unsigned short leverArmY0; // cm
   unsigned short leverArmZ0; // cm
   unsigned char sensType1;
   unsigned char reserved2_1;
   unsigned short leverArmX1; // cm
   unsigned short leverArmY1; // cm
   unsigned short leverArmZ1; // cm
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_esfla;

typedef struct __SET_GET_CFG__ESFLA{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x2f;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_esfla;

typedef struct __STR_CFG_ESFLA{
   const char *version="Message version (0 for this version):";
   const char *numConfigs="Number of configurations:";
   const char *sensType="Type of sensor(0:GNSS antenna; 1: accelerometer):";
   const char *leverArmX="x coordinate of sensor in the vehicle frame:";
   const char *leverArmY="y coordinate of sensor in the vehicle frame:";
   const char *leverArmZ="z coordinate of sensor in the vehicle frame:";
}__attribute__((packed))t_str_cfg_esfla;

//UBX CFG ESFWT
typedef struct __CFG_ESFWT_FLAGS{
   unsigned char combineTicks:1;
   unsigned char reserved1:3;
   unsigned char useWtSpeed:1;
   unsigned char dirPinPol:1;
   unsigned char useWtPin:1;
   unsigned char reserved2:1;
}t_cfg_esfwt_flags;

typedef struct __SET_GET_CFG_ESFWT{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x82;
   const unsigned short length = 32;
   unsigned char version;
   t_cfg_esfwt_flags flags;
   unsigned char reserved1[2];
   unsigned int wtFactor;
   unsigned int wtQuantError;
   unsigned int wtCountMax;
   unsigned short wtLatency;
   unsigned char wtFrequency;
   unsigned char reserved2;
   unsigned short speedDeadBand;
   unsigned char reserved3[10];
   unsigned short crc;
}__attribute__((packed)) t_set_get_cfg_esfwt; 

typedef struct __POLL_CFG_ESFWT{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x82;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_esfwt;

typedef struct __STR_CFG_ESFWT{
   const char *version="Message version(0 for this version):";
   const char *combineTicks="Use combined rear wheel ticks instead of the signle tick:";
   const char *useWtSpeed="Use speed measurements(data type 11 in ESF-MEAS) instead of single ticks(data type 10):";
   const char *dirPinPol="Direction pin polarity 0: High signal level means forward direction 1: High signal level means backward direction:";
   const char *useWtPin="Use whell tick pin for speed measurement:";
   const char *wtFactor="Wheel tick scale factor to obtain distance[m] from wheel ticks[0 = not set]:";
   const char *wtQuantError="Wheel tick quantization. If useWtSpeed is set then this is interpreted as the speed measurement error RMS<m or m/s>:";
   const char *wtCountMax="Wheel tick counter maximum value(rollover -1) (0 if no rollover but relative values). if useWtSpeed is set then this value is ignored:";
   const char *wtLatency="Wheel tick data latency due to e.g. CAN BUS:";
   const char *wtFrequency="Nominal wheel tick data frequency(0 = not set):";
   const char *speedDeadBand="Speed sensor dead band(0 = not set):";
}__attribute__((packed)) t_str_cfg_esfwt;


//UBX CFG MSG
typedef struct __POLL_CFG_MSG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x01;
   const unsigned short length = 2;
   unsigned char msgClass;
   unsigned char msgID;
   unsigned short crc;
}__attribute__((packed)) t_poll_cfg_msg; 

typedef struct __SET_CFG_MSG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x06;
   const unsigned char ID = 0x01;
   const unsigned short length = 8;
   unsigned char msgClass;
   unsigned char msgID;
   unsigned char rate[6];
   unsigned short crc;
}__attribute__((packed)) t_set_cfg_msg;

//UBX Reset Automatic sensor-mount alignemt
typedef struct __SET_ESF_RESETALG{
   const unsigned short header = htons((UBX_SYNCCHAR_ONE<<8)|UBX_SYNCCHAR_TWO);
   const unsigned char Class = 0x10;
   const unsigned char ID = 0x13;
   const unsigned short length = 0x00;
   unsigned short crc;
}__attribute__((packed)) t_set_esf_resetalg;


/************************************************************************************************************************************************/

#define STR_MODE_R	0x1                 // stream mode: read
#define STR_MODE_W	0x2                 // stream mode: write
#define STR_MODE_RW	0x3                 // stream mode: read/write

#define TAG_GNRMC	"RMC,"
#define TAG_GNGLL	"GLL,"
#define TAG_GNGGA	"GGA,"
#define TAG_GNVTG	"VTG,"
#define TAG_GNGSA	"GSA,"
#define TAG_XXGSV	"GSV,"

#define UBX_CLASS_OFFSET	0x02
#define UBX_LENGTH_OFFSET   	0x04
#define UBX_OVERHEAD		0x08
#define UBX_EXCLUDE_CRC_BYTES	0x04

#define UARTINTERFACE0	"ttyUSB0"
#define UARTINTERFACE1	"ttyUSB1"

#define POLL_CLASS01		0x01
#define POLL_CLASS10		0x10
#define POLL_ID02	     	0x02   
#define POLL_ID03	     	0x03   
#define POLL_ID05	     	0x05   
#define POLL_ID10		0x10
#define POLL_ID14		0x14
#define POLL_ID15		0x15
#define POLL_ID37		0x37

#define STR_DEBUG_SETTING	"SETTING"
#define STR_DEBUG_NMEA		"NMEA"
#define STR_DEBUG_MEASPEED	"SPEED"
#define STR_DEBUG_POS		"POS"
#define STR_DEBUG_CAN		"CAN"
#define STR_DEBUG_ESF_MEAS	"MEAS"
#define STR_DEBUG_ESF_RAW	"RAW"
#define STR_DEBUG_ESF_INS	"INS"
#define STR_DEBUG_ESF_ALG	"ALG"
#define STR_DEBUG_ESF_STATUS 	"STATUS"
#define STR_DEBUG_NAV_HNR	"HNR"
#define STR_DEBUG_NAV_ATT	"ATT"

#define SettingPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_SETTING), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define measpeedPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_MEASPEED), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define CANPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_CAN), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define GPSPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_POS), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define Print(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_NMEA), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define ATTPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_NAV_ATT), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define INSPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_ESF_INS), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define RAWPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_ESF_RAW), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define MEASPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_ESF_MEAS), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define ALGPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_ESF_ALG), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define STATUSPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_ESF_STATUS), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define HNRPrint(ID, Format, arg...) \
   if(strcmp(getenv(STR_DEBUG_NAV_HNR), "on") == 0)\
     fprintf(stdout, "<%s> " Format, ID, ##arg);

#define SOL_CAN_RAW (SOL_CAN_BASE + CAN_RAW)

#ifndef PF_CAN
/// Domain for CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
/// Family for CAN
#define AF_CAN PF_CAN
#endif

#define SPEED_RESOLUTION_MPS    (0.05625*1000.0/3600.0)
#define SPEED_RESOLUTION    	(0.05625)

/************************************************************************************************************************************************/
bool MainExit=false;

serial_t *pHandle = NULL;
tGNRMC gnrmc;
tGNGLL gngll;
tGNGGA gngga;
tGNVTG gnvtg;
tGNGSA gngsa;
tGXGSV gxgsv;

timer_t timerid;
/************************************************************************************************************************************************/
static int SetESFMEAS(serial_t *pHandle, double speed);
static int StartTimer(int alarmInterval, void (*SignalRoutine) (int signo));
static void StopTimer();


/************************************************************************************************************************************************/
static void ShellSignalHandle(int sigNum)
{
   StopTimer();
   printf("Signal NO. %d\n", sigNum);
   printf("Main Loop Exit!\n");
   MainExit = true;
}


static void ConfigureSignalRoutine(int sigNum)
{
   // printf("ConfigureSignalRoutine %d \n", sigNum);
   if(MainExit==false && pHandle != NULL){
     SetESFMEAS(pHandle, 100.0);
   }
}


static int StartTimer(int alarmInterval, void (*SignalRoutine) (int signo))
{
#if 0
     struct itimerval value;
     signal(SIGALRM,SignalRoutine);

     value.it_value.tv_sec = 0;			
     value.it_value.tv_usec = alarmInterval;			
     value.it_interval.tv_sec = 0;			
     value.it_interval.tv_usec = alarmInterval;	;			
     setitimer(ITIMER_REAL,&value,NULL);

     // alarm(sInterval);
     // ualarm(usInterval);

     d_printf(D_DEBUG, NULL, "\r\nPROSIX_TIMER1  sleep %d microsecond......\n",alarmInterval);
#endif
    struct sigaction act;  
    memset(&act, 0, sizeof(act));  
    act.sa_handler = SignalRoutine;  
    act.sa_flags = 0;

    sigemptyset(&act.sa_mask);  
  
    if (sigaction(SIGUSR1, &act, NULL) == -1)  
    {  
        printf("fail to sigaction\n"); 
        return -1; 
    }  

    struct sigevent evp;
    memset(&evp, 0, sizeof(struct sigevent));  
    evp.sigev_signo = SIGUSR1;  
    evp.sigev_notify = SIGEV_SIGNAL;  
    if (timer_create(CLOCK_REALTIME, &evp, &timerid) == -1)  
    {  
        printf("fail to timer_create\n");
        return -1;
    }  
  
    struct itimerspec it;  
    it.it_interval.tv_sec = 0;  
    it.it_interval.tv_nsec = alarmInterval*1000;
    it.it_value.tv_sec = 0;
    it.it_value.tv_nsec = alarmInterval*1000;  
    if (timer_settime(timerid, 0, &it, 0) == -1)  
    {  
        printf("fail to timer_settime\n");
        return -1;  
    }  

    return 0;
}

static void StopTimer()
{
    timer_delete(&timerid);
}


static serial_t *openserial(const char *path, int mode, char *msg)
{
    const int br[]={
        300,600,1200,2400,4800,9600,19200,38400,57600,115200,230400
    };
    serial_t *serial;
    int i,brate=9600,bsize=8,stopb=1;
    char *p,parity='N',dev[128],port[128],fctr[64]="";
    const speed_t bs[]={
        B300,B600,B1200,B2400,B4800,B9600,B19200,B38400,B57600,B115200,B230400
    };
    struct termios ios={0};
    int rw=0;

    printf("openserial: path=%s mode=%d\n",path,mode);
    
    if (!(serial=(serial_t *)malloc(sizeof(serial_t)))) return NULL;
    
    if (p=(char*)strchr(path,':')) {
        strncpy(port,path,p-path); port[p-path]='\0';
        sscanf(p,":%d:%d:%c:%d:%s",&brate,&bsize,&parity,&stopb,fctr);
    }
    else strcpy(port,path);

    printf("rate:%d, bitsize:%d, parity:%c, stopbit:%d, fctr:%s\n", brate, bsize, parity, stopb, fctr);
    
    for (i=0;i<11;i++) if (br[i]==brate) break;
    if (i>=12) {
        sprintf(msg,"bitrate error (%d)",brate);
        printf("openserial: %s path=%s\n",msg,path);
        free(serial);
        return NULL;
    }
    parity=(char)toupper((int)parity);
    
    sprintf(dev,"/dev/%s",port);
    
    if ((mode&STR_MODE_R)&&(mode&STR_MODE_W)) rw=O_RDWR;
    else if (mode&STR_MODE_R) rw=O_RDONLY;
    else if (mode&STR_MODE_W) rw=O_WRONLY;
    
    if ((serial->dev=open(dev,rw))<0) {// O_NOCTTY|O_NONBLOCK
        sprintf(msg,"device open error (%d)",errno);
        printf("openserial: %s dev=%s\n",msg,dev);
        free(serial);
        return NULL;
    }
    tcgetattr(serial->dev,&ios);
    //ios.c_iflag=0;
    //ios.c_oflag=0;
    //ios.c_lflag=0;     /* non-canonical */
    cfsetospeed(&ios,bs[i]);
    cfsetispeed(&ios,bs[i]);
    tcsetattr(serial->dev,TCSANOW,&ios);
    tcflush(serial->dev,TCIOFLUSH);

    tcgetattr(serial->dev,&ios);
    ios.c_cflag&=~CSIZE;
    ios.c_cflag|=bsize==7?CS7:CS8;

    switch (bsize) /*设置数据位数*/  
    {  
        case 7:  
            ios.c_cflag |= CS7;  
            break;  
        case 8:  
            ios.c_cflag |= CS8;  
            break;  
        default:  
            sprintf(msg,"Unsupported data size\n");  
            return NULL;  
    }  
    switch (parity)  
    {  
        case 'n':  
        case 'N':  
            ios.c_cflag &= ~PARENB;   /* Clear parity enable */  
            ios.c_iflag &= ~INPCK;     /* Enable parity checking */  
            break;  
        case 'o':  
        case 'O':  
            ios.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/   
            ios.c_iflag |= INPCK;             /* Disnable parity checking */  
            break;  
        case 'e':  
        case 'E':  
            ios.c_cflag |= PARENB;     /* Enable parity */  
            ios.c_cflag &= ~PARODD;   /* 转换为偶效验*/    
            ios.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':  
        case 's':  /*as no parity*/  
            ios.c_cflag &= ~PARENB;  
            ios.c_cflag &= ~CSTOPB;  
            break;  
        default:  
            sprintf(msg,"Unsupported parity\n");  
            return NULL;  
    }  
        /* 设置停止位*/     
    switch (stopb)  
    {  
        case 1:  
            ios.c_cflag &= ~CSTOPB;  
            break;  
        case 2:  
            ios.c_cflag |= CSTOPB;  
            break;  
        default:  
            fprintf(stderr,"Unsupported stop bits\n");  
            return NULL;  
    }  
    /* Set input parity option */  
    if (parity != 'n' && parity != 'N')  
       ios.c_iflag |= INPCK;  

    ios.c_cc[VTIME] = 5; // 15 seconds  
    ios.c_cc[VMIN] = 0;  

    // ios.c_cflag|=!strcmp(fctr,"rts")?CRTSCTS:0;
    ios.c_lflag&=~(ECHO | ICANON | ECHOE | ISIG);
    ios.c_oflag&=~OPOST;
    tcsetattr(serial->dev,TCSANOW,&ios);
    tcflush(serial->dev,TCIOFLUSH);

    return serial;
}
/* close serial --------------------------------------------------------------*/
static void closeserial(serial_t *serial)
{
    printf("closeserial: dev=%d\n",serial->dev);
    
    if (!serial) return;
    close(serial->dev);

    free(serial);
}
/* read serial ---------------------------------------------------------------*/
static int readserial(serial_t *serial, unsigned char *buff, int n, char *msg)
{
    int nr;
    memset(buff, 0, n);

    // printf("readserial: dev=%d n=%d\n",serial->dev,n);
    if (!serial) return 0;

    if ((nr=read(serial->dev,buff,n))<0) return 0;
    // printf("readserial: exit dev=%d nr=%d\n",serial->dev,nr);
    return nr;
}
/* write serial --------------------------------------------------------------*/
static int writeserial(serial_t *serial, unsigned char *buff, int n, char *msg)
{
    int ns;
    
    // printf("writeserial: dev=%d n=%d\n",serial->dev,n);
    
    if (!serial) return 0;
    if ((ns=write(serial->dev,buff,n))<0) return 0;

    // printf("writeserial: exit dev=%d ns=%d\n",serial->dev,ns);
    return ns;
}
/* get state serial ----------------------------------------------------------*/
static int stateserial(serial_t *serial)
{
    return !serial?0:(serial->error?-1:2);
}


/* init can socket ----------------------------------------------------------*/
static int CANInit(void)
{
    int FdSock, rc;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[1];
    can_err_mask_t err_mask = CAN_ERR_MASK;
    int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
    int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
    int nbytes;

    FdSock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(FdSock < 0)
    {
        perror("apply socket error");
        return -1;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(FdSock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
//    addr.can_ifindex = 0;

    rc = bind(FdSock, (struct sockaddr *)&addr, sizeof(addr));
    if(rc)
    {
        perror("bind socket failed");
        return -1;
    }

    rfilter[0].can_id   = TCU_268;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rc = setsockopt(FdSock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    if(rc)
    {
	perror("set filter failed");
	return -1;
    }

    rc = setsockopt(FdSock, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
    if(rc)
    {
        perror("set error filter failed");
        return -1;
    }

    rc = setsockopt(FdSock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
    if(rc)
    {
        perror("disable loopback failed");
	return -1;
    }

    rc = setsockopt(FdSock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
    if(rc)
    {
        perror("disable receive oown message failed");
	return -1;
    }

    return FdSock;
}

/* can dump ----------------------------------------------------------*/
static void can_dump(rcv_buff_t rcv_buff)
{
    CANPrint(STR_DEBUG_CAN,
        "\n\tID: %xh  %s  %s Len: %d  Data: %02x %02x %02x %02x %02x %02x %02x %02x",
    	rcv_buff.frame_ext.extended? rcv_buff.frame_ext.can_id|(rcv_buff.frame_ext.can_id_ext<<11) :rcv_buff.frame_ext.can_id,
    	rcv_buff.frame_ext.rt_request?"remote request":"not remote request",
    	rcv_buff.frame_ext.extended?"extended":"standard",
    	rcv_buff.frame_ext.can_dlc,
    	rcv_buff.frame_ext.data.payload[0],
    	rcv_buff.frame_ext.data.payload[1],
    	rcv_buff.frame_ext.data.payload[2],
    	rcv_buff.frame_ext.data.payload[3],
    	rcv_buff.frame_ext.data.payload[4],
    	rcv_buff.frame_ext.data.payload[5],
    	rcv_buff.frame_ext.data.payload[6],
    	rcv_buff.frame_ext.data.payload[7]);

    switch (rcv_buff.frame_ext.can_id)
    {
        case TCU_268:
	    CANPrint(STR_DEBUG_CAN,
                   "\n\tAT_HeavyDeleclerationFlag %d"
                   "\n\tAT_ActualGear %d"
                   "\n\tAT_VehicleSpeed %lf\n",
                rcv_buff.frame_ext.data.tcu_268.AT_HeavyDeleclerationFlag,
                rcv_buff.frame_ext.data.tcu_268.AT_ActualGear,
                (((unsigned short)rcv_buff.frame_ext.data.tcu_268.AT_VehicleSpeed_MSB<<8)|rcv_buff.frame_ext.data.tcu_268.AT_VehicleSpeed_LSB)*SPEED_RESOLUTION);

	break;

        default:
            CANPrint(STR_DEBUG_CAN, "unexpect CAN frame\n");
    }


}

/* receive the can message ----------------------------------------------------------*/
static void* ReceiveCANMessage(void *pHandle)
{

    int FdSock, rc;
    struct sockaddr_can addr;
    struct ifreq ifr;
    rcv_buff_t rcv_buff;
    socklen_t len = sizeof(addr);
    int nbytes;
    
    FdSock = CANInit();
    if(FdSock < 0)
    {
	perror("open can socket failed");
	return NULL;
    }


    while(!MainExit)
    {

        nbytes = recvfrom(FdSock, &rcv_buff.frame, sizeof(rcv_buff.frame),
                      0, (struct sockaddr*)&addr, &len);

        /* get interface name of the received CAN frame */
        ifr.ifr_ifindex = addr.can_ifindex;
        ioctl(FdSock, SIOCGIFNAME, &ifr);
        // printf("\nReceived a CAN frame from interface %s\n", ifr.ifr_name);

	if (nbytes < 0){
            perror("can raw socket read");
            return NULL;
        }
	
        if (nbytes != sizeof(rcv_buff.frame)){
            fprintf(stderr, "read: incomplete CAN frame\n");
            return NULL;
        }

	//dump CAN ID |CAN LEN |CAN DATA
        can_dump(rcv_buff);
        
        // Update Store
        switch (rcv_buff.frame.can_id){
            case TCU_268:{
                   double speed =(((unsigned short)rcv_buff.frame_ext.data.tcu_268.AT_VehicleSpeed_MSB<<8)|rcv_buff.frame_ext.data.tcu_268.AT_VehicleSpeed_LSB)*SPEED_RESOLUTION_MPS;
                   if(NULL != pHandle){
                      (void)SetESFMEAS((serial_t*)pHandle, speed);
                   }
                }
                break;
            default:
                fprintf(stderr, "unexpect CAN frame\n");
        }
    }
}

/* read the vehicle speed with the PCAN--------------------------------------*/
static void* ReceiveCANMessagewithPCAN(void *pHandle)
{
   TPCANMsg Message;
   TPCANStatus Status;
   unsigned long ulIndex = 0;
   fd_set Fds;

   Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
   printf("Initialize CAN: %i\n", (int) Status);

   int fd;
   CAN_GetValue(PCAN_USBBUS1, PCAN_RECEIVE_EVENT, &fd,sizeof(int));
   printf("Get CAN Value: %i\n", (int) fd);

   /* Watch stdin (fd 0) to see when it has input. */
   FD_ZERO(&Fds);
   FD_SET(fd, &Fds);

   while (!MainExit && select(fd+1, &Fds, NULL, NULL, NULL) > 0) {
      Status = CAN_Read(PCAN_USBBUS1, &Message, NULL);
      if (Status != PCAN_ERROR_OK) {
              printf("Error 0x%x\n", (int) Status);
              break;
      }
      if(Message.ID == 0x268){
         union can_data *data = (union can_data*)Message.DATA;

	 double speed_mps =(((unsigned short)data->tcu_268.AT_VehicleSpeed_MSB<<8)|data->tcu_268.AT_VehicleSpeed_LSB)*SPEED_RESOLUTION_MPS;
	 double speed_kmph =(((unsigned short)data->tcu_268.AT_VehicleSpeed_MSB<<8)|data->tcu_268.AT_VehicleSpeed_LSB)*SPEED_RESOLUTION;

         CANPrint(STR_DEBUG_CAN, "  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                         (int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
                         (int) Message.DATA[1], (int) Message.DATA[2],
                         (int) Message.DATA[3], (int) Message.DATA[4],
                         (int) Message.DATA[5], (int) Message.DATA[6],
                         (int) Message.DATA[7]);
         CANPrint(STR_DEBUG_CAN, 
           	   "\n\tAT_HeavyDeleclerationFlag %d"
                         "\n\tAT_ActualGear %d"
                         "\n\tAT_VehicleSpeed %lfkm/h, %lfm/s\n",
                         data->tcu_268.AT_HeavyDeleclerationFlag,
                         data->tcu_268.AT_ActualGear,
                         speed_kmph,
                         speed_mps);

         if(NULL != pHandle){
            (void)SetESFMEAS((serial_t*)pHandle, speed_mps);
         }


      }
   }
   return 0;
}


/* parse the gnrmc ----------------------------------------------------------*/
static bool GNRMC(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   double utctime;
   int len = sizeof(gnrmc.ID);

   memcpy(gnrmc.ID, buff, len-1);
   gnrmc.ID[len-1] = '\0';
   num  -= len;
   buff += len;

   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
             utctime = strtod(pbuff, NULL);
             gnrmc.hour = (int)utctime/10000;
             gnrmc.miniute = (int)utctime/100%100;
             gnrmc.second = fmod(utctime, 100);
             Print(gnrmc.ID, "UTC Time: %d-%d-%lf\n", gnrmc.hour, gnrmc.miniute, gnrmc.second);
             break;
          case 1:
             if(isalpha(pbuff[0])) {
                gnrmc.isVailed = pbuff[0];
                Print(gnrmc.ID, "IS Vailed: %c\n", gnrmc.isVailed);
	     }
             break;
          case 2:
             gnrmc.latitude = strtod(pbuff, NULL);
             Print(gnrmc.ID, "Latitude: %lf\n", gnrmc.latitude);
             break;
          case 3:
             if(isalpha(pbuff[0])) {
                gnrmc.southorNorth = pbuff[0];
                Print(gnrmc.ID, "South or North: %c\n", gnrmc.southorNorth);
             }
             break;
          case 4:
             gnrmc.longitude = strtod(pbuff, NULL);
             Print(gnrmc.ID, "Longitude: %lf\n", gnrmc.longitude);
             break;
          case 5:
             if(isalpha(pbuff[0])) {
                gnrmc.eastorWest = pbuff[0];
                Print(gnrmc.ID, "East or West: %c\n", gnrmc.eastorWest);
             }
             break;
          case 6:
             gnrmc.speedk = strtod(pbuff, NULL);
             Print(gnrmc.ID, "Speed: %lf\n", gnrmc.speedk);
             break;
          case 7:
             gnrmc.heading = strtod(pbuff, NULL);
             Print(gnrmc.ID, "Heading: %lf\n", gnrmc.heading);
             break;
          case 8:
             gnrmc.year = (int)strtol(pbuff, NULL, 10);
             gnrmc.day=gnrmc.year/10000;
             gnrmc.month=gnrmc.year/100%100;
             gnrmc.year=gnrmc.year%100;
             Print(gnrmc.ID, "Date: %d-%d-%d\n", gnrmc.year, gnrmc.month, gnrmc.day);
             break;
          case 9:
             gnrmc.declination = strtod(pbuff, NULL);
             Print(gnrmc.ID, "Declination: %lf\n", gnrmc.declination);
             break;
          case 10:
             if(isalpha(pbuff[0])) {
                gnrmc.direction = pbuff[0];
                Print(gnrmc.ID, "Dircetion: %c\n", gnrmc.direction);
             }
             break;
          case 11:
             if(isalpha(pbuff[0])) {
                gnrmc.mode = pbuff[0];
                Print(gnrmc.ID, "Mode: %c\n", gnrmc.mode);
             }
             break;
       }
   }

   double latitude  = (unsigned int)(gnrmc.latitude/100)  + (gnrmc.latitude  - (unsigned int)(gnrmc.latitude/100)*100)/60;
   double longitude = (unsigned int)(gnrmc.longitude/100) + (gnrmc.longitude - (unsigned int)(gnrmc.longitude/100)*100)/60;
   GPSPrint(gnrmc.ID, "%lf, %lf\n" ,latitude, longitude);
}


/* parse the gngll ----------------------------------------------------------*/
static bool GNGLL(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   double utctime;
   int len = sizeof(gngll.ID);

   memcpy(gngll.ID, buff, len-1);
   gngll.ID[len-1] = '\0';
   num  -= len;
   buff += len;

   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
             gngll.latitude = strtod(pbuff, NULL);
             Print(gngll.ID, "Latitude: %lf\n", gngll.latitude);
             break;
          case 1:
             if(isalpha(pbuff[0])) {
                gngll.southorNorth = pbuff[0];
                Print(gngll.ID, "South or North: %c\n", gngll.southorNorth);
             }
             break;
          case 2:
             gngll.longitude = strtod(pbuff, NULL);
             Print(gngll.ID, "Longitude: %lf\n", gngll.longitude);
             break;
          case 3:
             if(isalpha(pbuff[0])) {
                gngll.eastorWest = pbuff[0];
                Print(gngll.ID, "East or West: %c\n", gngll.eastorWest);
             }
             break;
          case 4:
             utctime = strtod(pbuff, NULL);
             gngll.hour = (int)utctime/10000;
             gngll.miniute = (int)utctime/100%100;
             gngll.second = fmod(utctime, 100);
             Print(gngll.ID, "UTC Time: %d-%d-%lf\n", gngll.hour, gngll.miniute, gngll.second);
             break;
          case 5:
             if(isalpha(pbuff[0])) {
                gngll.isVailed = pbuff[0];
                Print(gngll.ID, "IS Vailed: %c\n", gngll.isVailed);
             }
             break;
          case 6:
             if(isalpha(pbuff[0])) {
                gngll.mode = pbuff[0];
                Print(gngll.ID, "Mode: %c\n", gngll.mode);
             }
             break;
       }
   }


}

/* parse the gngga ----------------------------------------------------------*/
static bool GNGGA(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   double utctime;
   int len = sizeof(gngga.ID);

   memcpy(gngga.ID, buff, len-1);
   gngga.ID[len-1] = '\0';
   num  -= len;
   buff += len;

   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
             utctime = strtod(pbuff, NULL);
             gngga.hour = (int)utctime/10000;
             gngga.miniute = (int)utctime/100%100;
             gngga.second = fmod(utctime, 100);
             Print(gngga.ID, "UTC Time: %d-%d-%lf\n", gngga.hour, gngga.miniute, gngga.second);
             break;
          case 1:
             gngga.latitude = strtod(pbuff, NULL);
             Print(gngga.ID, "Latitude: %lf\n", gngga.latitude);
             break;
          case 2:
             if(isalpha(pbuff[0])) {
                gngga.southorNorth = pbuff[0];
                Print(gngga.ID, "South or North: %c\n", gngga.southorNorth);
             }
             break;
          case 3:
             gngga.longitude = strtod(pbuff, NULL);
             Print(gngga.ID, "Longitude: %lf\n", gngga.longitude);
             break;
          case 4:
             if(isalpha(pbuff[0])) {
                gngga.eastorWest = pbuff[0];
                Print(gngga.ID, "East or West: %c\n", gngga.eastorWest);
             }
             break;
          case 5:
             gngga.status = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gngga.ID, "Status: %d\n", gngga.status);
             break;
          case 6:
             gngga.numSatellite = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gngga.ID, "Satellite Number %d\n", gngga.numSatellite);
             break;
          case 7:
             gngga.hdop = strtod(pbuff, NULL);
             Print(gngga.ID, "HDOP %lf\n", gngga.hdop);
             break;
          case 8:
             gngga.altitude = strtod(pbuff, NULL);
             break;
          case 9:
             if(isalpha(pbuff[0])) {
                gngga.altUnit = pbuff[0];
                Print(gngga.ID, "Altitude %lf %c\n", gngga.altitude, gngga.altUnit);
             }
             break;
          case 10:
             gngga.geoid = strtod(pbuff, NULL);
             break;
          case 11:
             if(isalpha(pbuff[0])) {
                gngga.geoUnit = pbuff[0];
                Print(gngga.ID, "Geoid %lf %c\n", gngga.geoid, gngga.geoUnit);
             }
             break;
          case 12:
             gngga.diffSec = (int)strtod(pbuff, NULL);
             Print(gngga.ID, "Diff Time(second): %lf\n", gngga.diffSec);
             break;
          case 13:
             gngga.numBaseStation = (int)strtol(pbuff, NULL, 10);
             Print(gngga.ID, "Base Station Number: %d\n", gngga.numBaseStation);
             break;
       }
   }


}


/* parse the gngvt ----------------------------------------------------------*/
static bool GNVTG(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   int len = sizeof(gnvtg.ID);

   memcpy(gnvtg.ID, buff, len-1);
   gnvtg.ID[len-1] = '\0';
   num  -= len;
   buff += len;

   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
             gnvtg.rheading = strtod(pbuff, NULL);
             break;
          case 1:
             if(isalpha(pbuff[0])) {
                gnvtg.rhUnit = pbuff[0];
                Print(gnvtg.ID, "Heading to real north: %lf %c\n", gnvtg.rheading, gnvtg.rhUnit);
             }
             break;
          case 2:
             gnvtg.mheading = strtod(pbuff, NULL);
             break;
          case 3:
             if(isalpha(pbuff[0])) {
                gnvtg.mhUnit = pbuff[0];
                Print(gnvtg.ID, "Heading to magnetic north: %lf %c\n", gnvtg.mheading, gnvtg.mhUnit);
             }
          break;
          case 4:
             gnvtg.kspeed = strtod(pbuff, NULL);
             break;
          case 5:
             if(isalpha(pbuff[0])) {
                gnvtg.ksUnit = pbuff[0];
                Print(gnvtg.ID, "Speed (knots): %lf %c\n", gnvtg.kspeed, gnvtg.ksUnit);
             }
             break;
          case 6:
             gnvtg.hspeed = strtod(pbuff, NULL);
             break;
          case 7:
             if(isalpha(pbuff[0])) {
                gnvtg.hsUnit = pbuff[0];
                Print(gnvtg.ID, "Speed (km/h): %lf %c\n", gnvtg.hspeed, gnvtg.hsUnit);
             }
             break;
          case 8:
             if(isalpha(pbuff[0])) {
                gnvtg.mode = pbuff[0];
                Print(gnvtg.ID, "Mode: %c\n", gnvtg.mode);
             }
             break;
      }
   }
}


/* parse the gngsa ----------------------------------------------------------*/
static bool GNGSA(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   int len = sizeof(gngsa.ID);

   memcpy(gngsa.ID, buff, len-1);
   gngsa.ID[len-1] = '\0';
   num  -= len;
   buff += len;
   

   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
             if(isalpha(pbuff[0])) {
                gngsa.mode = pbuff[0];
                Print(gngsa.ID, "MOde: %c\n", gngsa.mode);
             }
             break;
          case 1:
             gngsa.type = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gngsa.ID, "Type: %d\n", gngsa.type);
             break;
          case 2:
          case 3:
          case 4:
          case 5:
          case 6:
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          case 13:
             gngsa.prn[j-3] = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gngsa.ID, "pnr[%d]: %d\n", j-3, gngsa.prn[j-3]);
             break;
          case 14:
             gngsa.pdop = strtod(pbuff, NULL);
             Print(gngsa.ID, "PDOP: %lf\n", gngsa.pdop);
          break;
          case 15:
             gngsa.hdop = strtod(pbuff, NULL);
             Print(gngsa.ID, "HDOP: %lf\n", gngsa.hdop);
             break;
          case 16:
             gngsa.vdop = strtod(pbuff, NULL);
             Print(gngsa.ID, "VPDOP: %lf\n", gngsa.vdop);
             break;
      }
   }
}


/* parse the gxgsv ----------------------------------------------------------*/
static bool GXGSV(char *buff, unsigned int num)
{
   const char *pbuff;
   const char *delim = ",";
   int j = 0;
   int len = sizeof(gxgsv.ID);

   memcpy(gxgsv.ID, buff, len-1);
   gxgsv.ID[len-1] = '\0';
   num  -= len;
   buff += len;
   
   while((pbuff = strsep(&buff, delim)) != NULL)
   {
       switch(j++){
          case 0:
	     gxgsv.amount = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "Amount: %d\n", gxgsv.amount);
             break;
          case 1:
             gxgsv.num = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "Number: %d\n", gxgsv.num);
             break;
	  case 2:
	     gxgsv.numSatellite = (unsigned char)strtol(pbuff, NULL, 10);
	     Print(gxgsv.ID, "numSatellite: %d\n", gxgsv.numSatellite);
	     break;
          case 3:
             gxgsv.satellite[0].prn = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]PRN: %d\n", (gxgsv.num-1)*4+1, gxgsv.satellite[0].prn);
             break;
          case 4:
             gxgsv.satellite[0].elevation = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Elevation: %d\n", (gxgsv.num-1)*4+1, gxgsv.satellite[0].elevation);
             break;
          case 5:
             gxgsv.satellite[0].azimuth = (unsigned short)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Azimuth: %d\n", (gxgsv.num-1)*4+1, gxgsv.satellite[0].azimuth);
             break;
          case 6:
             gxgsv.satellite[0].snr = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]SNR: %d\n", (gxgsv.num-1)*4+1, gxgsv.satellite[0].snr);
             break;

          case 7:
             gxgsv.satellite[1].prn = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]PRN: %d\n", (gxgsv.num-1)*4+2, gxgsv.satellite[1].prn);
             break;
          case 8:
             gxgsv.satellite[1].elevation = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Elevation: %d\n", (gxgsv.num-1)*4+2, gxgsv.satellite[1].elevation);
             break;
          case 9:
             gxgsv.satellite[1].azimuth = (unsigned short)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Azimuth: %d\n", (gxgsv.num-1)*4+2, gxgsv.satellite[1].azimuth);
             break;
          case 10:
             gxgsv.satellite[1].snr = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]SNR: %d\n", (gxgsv.num-1)*4+2, gxgsv.satellite[1].snr);
             break;

          case 11:
             gxgsv.satellite[2].prn = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]PRN: %d\n", (gxgsv.num-1)*4+3, gxgsv.satellite[2].prn);
             break;
          case 12:
             gxgsv.satellite[2].elevation = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Elevation: %d\n", (gxgsv.num-1)*4+3, gxgsv.satellite[2].elevation);
             break;
          case 13:
             gxgsv.satellite[2].azimuth = (unsigned short)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Azimuth: %d\n", (gxgsv.num-1)*4+3, gxgsv.satellite[2].azimuth);
             break;
          case 14:
             gxgsv.satellite[2].snr = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]SNR: %d\n", (gxgsv.num-1)*4+3, gxgsv.satellite[2].snr);
             break;

          case 15:
             gxgsv.satellite[3].prn = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]PRN: %d\n", (gxgsv.num-1)*4+4, gxgsv.satellite[3].prn);
             break;
          case 16:
             gxgsv.satellite[3].elevation = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Elevation: %d\n", (gxgsv.num-1)*4+4, gxgsv.satellite[3].elevation);
             break;
          case 17:
             gxgsv.satellite[3].azimuth = (unsigned short)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]Azimuth: %d\n", (gxgsv.num-1)*4+4, gxgsv.satellite[3].azimuth);
             break;
          case 18:
             gxgsv.satellite[3].snr = (unsigned char)strtol(pbuff, NULL, 10);
             Print(gxgsv.ID, "[%d]SNR: %d\n", (gxgsv.num-1)*4+4, gxgsv.satellite[3].snr);
             break;

      }
   }
}

static unsigned short ubxCrc(const unsigned char* data, unsigned int size)
{
   unsigned int crc_a = 0;
   unsigned int crc_b = 0;
   if(size > 0)
   {
      do {
         crc_a += *data++;
         crc_b += crc_a;
      }while(--size);
      crc_a &= 0xff;
      crc_b &= 0xff;
   }
   return (unsigned short)(crc_a|(crc_b<<8));
}


static void printMeasurementSpeedFeed(unsigned char *pdata, int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_input_esf_meas *esf_meas= (t_input_esf_meas*)ubxFrame;
   t_str_input_esf_meas str_esf_meas;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.timeTag,	 		esf_meas->timeTag);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.timeMarkSent, 		esf_meas->flags.timeMarkSent);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.timeMarkEdge, 		esf_meas->flags.timeMarkEdge);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.calibTtagValid, 		esf_meas->flags.calibTtagValid);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.id,		 		esf_meas->id);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.dataField,		 	esf_meas->data.dataField);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_esf_meas.dataType,		 	esf_meas->data.dataType);

//   for(int k=0;k<ubxFrameMaxLen;k++){
//     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%02x ", ubxFrame[k]);
//     if(k%16 == 15) offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
//   }
//   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");

   measpeedPrint("MEAS Speed Feed ", "%s", printbuff);               
   delete[] printbuff;
}

/*Set ESF MEAS**********************************************************************************************/
static int SetESFMEAS(serial_t *pHandle, double speed)
{
   int rslt, i;
   char msg[256];
   static unsigned short id = 0;
   static unsigned int timeTag = 0;
   t_input_esf_meas esf_meas;

  
  timeTag+=1;
  esf_meas.timeTag = timeTag;
  esf_meas.id = id;
  esf_meas.data.dataType = 11;
  esf_meas.data.dataField = (unsigned int)(speed*1e3)&0x0ffffff;
  esf_meas.crc = ubxCrc((unsigned char *)&esf_meas+UBX_CLASS_OFFSET, sizeof(t_input_esf_meas)-UBX_CLASS_OFFSET-2);

  printMeasurementSpeedFeed((unsigned char*)&esf_meas, sizeof(t_input_esf_meas));

  rslt = writeserial(pHandle, (unsigned char *)&esf_meas, sizeof(t_input_esf_meas), msg);
  if(rslt <=0){
     printf("Set ESF MEAS error %s\n", msg);
     return rslt;
  }
  return rslt;
}


/*set cfg msg***************************************************************************************************/
static int SetCFGMSG(serial_t *pHandle, unsigned char *rate, unsigned char size, unsigned char Class, unsigned char ID)
{
   int rslt, i;
   char msg[256];
   t_set_cfg_msg cfg_msg; 

  cfg_msg.msgClass = Class;
  cfg_msg.msgID = ID;
  for(i=0;i<size&&rate!=NULL;i++){
     cfg_msg.rate[i] = rate[i];
  }
  cfg_msg.crc = ubxCrc((unsigned char *)&cfg_msg+UBX_CLASS_OFFSET, sizeof(t_set_cfg_msg) - UBX_EXCLUDE_CRC_BYTES);

  // printf("crc %x\n"    , cfg_msg.crc);
  // unsigned char* pdata = (unsigned char *)&cfg_msg;
  // for(i=0;i<sizeof(cfg_msg);i++){
  //     printf("%02x ", *pdata++);
  // }
  // printf("\n");
  rslt = writeserial(pHandle, (unsigned char *)&cfg_msg, sizeof(cfg_msg), msg);
  if(rslt <=0){
     printf("SetCFGMSG error %s\n", msg);
     return rslt;
  }
  return rslt;
}

/*get cfg msg***************************************************************************************************/
static int GetCFGMSG(serial_t *pHandle, unsigned char *rate, unsigned char size, unsigned char Class, unsigned char ID)
{
   int rslt, i;
   int frameLen = 0, frameMaxLen = 0, num = 0;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[256];
   unsigned char frame[256];
   t_poll_cfg_msg poll_cfg_msg;
   t_set_cfg_msg  *set_cfg_msg;

   poll_cfg_msg.msgClass = Class;
   poll_cfg_msg.msgID = ID;
   poll_cfg_msg.crc = ubxCrc((unsigned char *)&poll_cfg_msg+UBX_CLASS_OFFSET, sizeof(poll_cfg_msg)-UBX_CLASS_OFFSET-2);

  //  printf("crc %x\n"    , poll_cfg_msg.crc);
  //  unsigned char* pdata = (unsigned char *)&poll_cfg_msg;
  //  for(i=0;i<sizeof(poll_cfg_msg);i++){
  //      printf("%02x ", *pdata++);
  //  }
  //  printf("\n");

   rslt = writeserial(pHandle, (unsigned char *)&poll_cfg_msg, sizeof(poll_cfg_msg), msg);
   if(rslt <=0){
      printf("SetCFGMSG error %s\n", msg);
      return rslt;
   }

   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(int j=0;j<num;j++){
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }

         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != poll_cfg_msg.Class || frame[3] != poll_cfg_msg.ID)){
               frameStart = false;
               continue;
            }

            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }

         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);

            set_cfg_msg = (t_set_cfg_msg*)frame;
            for(i=0;i<size&&rate!=NULL;i++){
               rate[i] = set_cfg_msg->rate[i];
            }

            return frameLen;
         }

   
      }
   }

 return rslt;

}

/*print cfg hnr***************************************************************************************************/
static void printCFGHNR(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_hnr *cfg_hnr= (t_set_get_cfg_hnr*)ubxFrame;
   t_str_cfg_hnr str_cfg_hnr;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_hnr.highNavRate, 		cfg_hnr->highNavRate);
   SettingPrint("CFG HNR<06,5C>", "%s", printbuff);
   delete[] printbuff;
}

/*set cfg hnr***************************************************************************************************/
static int Set_CFG_HNR(serial_t *pHandle, unsigned char* buff, unsigned int rate)
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_hnr *cfg_hnr = (t_set_get_cfg_hnr*)buff;
   
   cfg_hnr->highNavRate = rate;
   cfg_hnr->crc = ubxCrc((unsigned char*)cfg_hnr+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_hnr)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_hnr, sizeof(t_set_get_cfg_hnr), msg);
   if(rslt <=0){
      printf("Set CFG HNR error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*get cfg hnr***************************************************************************************************/
static int Get_CFG_HNR(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_hnr cfg_hnr;

   cfg_hnr.crc = ubxCrc((unsigned char*)&cfg_hnr+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_hnr) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_HNR crc %04x\n"    , cfg_hnr.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_hnr;
   // for(i=0;i<sizeof(t_poll_cfg_hnr;i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_hnr, sizeof(t_poll_cfg_hnr), msg);
   if(rslt <=0){
      printf("Get CFG HNR error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_hnr.Class || frame[3] != cfg_hnr.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printCFGHNR(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}


/*print cfg rate***************************************************************************************************/
static void printCFGRate(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_rate *cfg_rate= (t_set_get_cfg_rate*)ubxFrame;
   t_str_cfg_rate str_cfg_rate;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_rate.measRate, 		cfg_rate->measRate);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_rate.navRate, 		cfg_rate->navRate);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_rate.timeRef, 		cfg_rate->timeRef);
   SettingPrint("CFG Rate<06,08>", "%s", printbuff);
   delete[] printbuff;
}

/*set cfg rate***************************************************************************************************/
static int Set_CFG_Rate(serial_t *pHandle, unsigned char* buff, unsigned int rate)
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_rate *cfg_rate = (t_set_get_cfg_rate*)buff;
   
   cfg_rate->measRate = rate;
   cfg_rate->navRate = 1;
   cfg_rate->crc = ubxCrc((unsigned char*)cfg_rate+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_rate)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_rate, sizeof(t_set_get_cfg_rate), msg);
   if(rslt <=0){
      printf("Set CFG Rate error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*get cfg rate***************************************************************************************************/
static int Get_CFG_Rate(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_rate cfg_rate;

   cfg_rate.crc = ubxCrc((unsigned char*)&cfg_rate+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_rate) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_RATE crc %04x\n"    , cfg_rate.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_rate;
   // for(i=0;i<sizeof(t_poll_cfg_rate;i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_rate, sizeof(t_poll_cfg_rate), msg);
   if(rslt <=0){
      printf("Get CFG Rate error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_rate.Class || frame[3] != cfg_rate.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printCFGRate(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}


/*print cfg navx5***************************************************************************************************/
static void printNAVX5(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_navx5 *cfg_navx5= (t_set_get_cfg_navx5*)ubxFrame;
   t_str_cfg_navx5 str_cfg_navx5;

   if(cfg_navx5->mask1.minMax){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.minSVs, 	cfg_navx5->minSVs);
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.maxSVs, 	cfg_navx5->maxSVs);
   }
   else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.minMax, 	"OFF");
   }

   if(cfg_navx5->mask1.minCn0){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.minCN0, 	cfg_navx5->minCN0);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.minCn0, 	"OFF");
   }

   if(cfg_navx5->mask1.initial3dfix){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.iniFix3D, 	cfg_navx5->iniFix3D);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.initial3dfix, 	"OFF");
   }


   if(cfg_navx5->mask1.wknRoll){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.wknRollover, 	cfg_navx5->wknRollover);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.wknRoll, 	"OFF");
   }

   if(cfg_navx5->mask1.ackAid){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.ackAiding, 	cfg_navx5->ackAiding);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.ackAid, 	"OFF");
   }

   if(cfg_navx5->mask1.ppp){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.usePPP, 	cfg_navx5->usePPP);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.ppp,	 	"OFF");
   }

   if(cfg_navx5->mask1.aop){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.aopCfg, 	cfg_navx5->aopCfg.useAOP);
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.aopOrbMaxErr, 	cfg_navx5->aopOrbMaxErr);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.aop,	 	"OFF");
   }

   if(cfg_navx5->mask2.adr){
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_navx5.useAdr, 	cfg_navx5->useAdr);
   }else{
      offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_navx5.adr,	 	"OFF");
   }

   SettingPrint("CFG NAVX5 <06,23>", "%s", printbuff);
   delete[] printbuff;
}

/*set cfg navx5***************************************************************************************************/
static int Set_CFG_NAVX5(serial_t *pHandle, unsigned char* buff, bool enableAdr)
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_navx5 *cfg_navx5 = (t_set_get_cfg_navx5*)buff;
   
   if(enableAdr == true) {cfg_navx5->mask2.adr = 1; cfg_navx5->useAdr = 1;}
   else {cfg_navx5->mask2.adr = 0; cfg_navx5->useAdr = 0;};
   // printf("==mask2 %x  useAdr %x \n", cfg_navx5->mask2, cfg_navx5->useAdr);
   cfg_navx5->crc = ubxCrc((unsigned char*)cfg_navx5+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_navx5)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_navx5, sizeof(t_set_get_cfg_navx5), msg);
   if(rslt <=0){
      printf("Set CFG NAVX5 error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*get cfg navx5***************************************************************************************************/
static int Get_CFG_NAVX5(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_navx5 cfg_navx5;

   cfg_navx5.crc = ubxCrc((unsigned char*)&cfg_navx5+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_navx5) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_NAVX5 crc %04x\n"    , cfg_navx5.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_navx5;
   // for(i=0;i<sizeof(t_poll_cfg_navx5;i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_navx5, sizeof(t_poll_cfg_navx5), msg);
   if(rslt <=0){
      printf("Get CFG NAVX5 error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_navx5.Class || frame[3] != cfg_navx5.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printNAVX5(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}


/*print cfg esfalg***************************************************************************************************/
static void printESFALG(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_esfalg *cfg_esfalg= (t_set_get_cfg_esfalg*)ubxFrame;
   t_str_cfg_esfalg str_cfg_esfalg;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfalg.version, 		cfg_esfalg->bitfield.version);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_esfalg.doAutoMntAlg, 	cfg_esfalg->bitfield.doAutoMntAlg?"Y":"N");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfalg.yaw, 		cfg_esfalg->yaw);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfalg.pitch, 		cfg_esfalg->pitch);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfalg.roll, 		cfg_esfalg->roll);
   SettingPrint("ESF ALG<06,56>", "%s", printbuff);
   delete[] printbuff;
}


/*set cfg esfalg***************************************************************************************************/
static int SetESFALG(serial_t *pHandle, unsigned char* buff, bool enable, int angle[])
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_esfalg *cfg_esfalg = (t_set_get_cfg_esfalg*)buff;
   
   if(enable == true) cfg_esfalg->bitfield.doAutoMntAlg = 1;
   else cfg_esfalg->bitfield.doAutoMntAlg = 0;
   cfg_esfalg->yaw = angle[0];
   cfg_esfalg->pitch = angle[1];
   cfg_esfalg->roll = angle[2];
   cfg_esfalg->crc = ubxCrc((unsigned char*)cfg_esfalg+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_esfalg)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_esfalg, sizeof(t_set_get_cfg_esfalg), msg);
   if(rslt <=0){
      printf("SetESFALG error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*get esf esfalg***************************************************************************************************/
static int GetESFALG(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_esfalg cfg_esfalg;

   cfg_esfalg.crc = ubxCrc((unsigned char*)&cfg_esfalg+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_esfalg) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_ESFLA crc %04x\n"    , cfg_esfalg.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_esfalg;
   // for(i=0;i<sizeof(t_poll_cfg_esfalg;i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_esfalg, sizeof(t_poll_cfg_esfalg), msg);
   if(rslt <=0){
      printf("GetESFWT error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_esfalg.Class || frame[3] != cfg_esfalg.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printESFALG(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}

/*print esf la***************************************************************************************************/
static void printESFLA(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_esfla *cfg_esfla= (t_set_get_cfg_esfla*)ubxFrame;
   t_str_cfg_esfla str_cfg_esfla;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfla.version, 		cfg_esfla->version);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfla.numConfigs, 	cfg_esfla->numConfigs);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t-----------------------------------------------------------------------------------\n");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.sensType, 	cfg_esfla->sensType0);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmX, 	cfg_esfla->leverArmX0);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmY, 	cfg_esfla->leverArmY0);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmZ, 	cfg_esfla->leverArmZ0);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t-----------------------------------------------------------------------------------\n");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.sensType, 	cfg_esfla->sensType1);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmX, 	cfg_esfla->leverArmX1);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmY, 	cfg_esfla->leverArmY1);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_cfg_esfla.leverArmZ, 	cfg_esfla->leverArmZ1);
   SettingPrint("ESF LA<06,2F>", "%s", printbuff);
   delete[] printbuff;
}

/*set cfg esfla***************************************************************************************************/
static int SetESFLA(serial_t *pHandle, unsigned char* buff, unsigned int antenna[], unsigned int accel[])
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_esfla *cfg_esfla = (t_set_get_cfg_esfla*)buff;
   
   cfg_esfla->numConfigs = 0x02;
   cfg_esfla->sensType0 = 0x00;
   cfg_esfla->leverArmX0 = antenna[0];
   cfg_esfla->leverArmY0 = antenna[1];
   cfg_esfla->leverArmZ0 = antenna[2];
   cfg_esfla->sensType1 = 0x01;
   cfg_esfla->leverArmX1 = accel[0];
   cfg_esfla->leverArmY1 = accel[1];
   cfg_esfla->leverArmZ1 = accel[2];
   cfg_esfla->crc = ubxCrc((unsigned char*)cfg_esfla+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_esfla)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_esfla, sizeof(t_set_get_cfg_esfla), msg);
   if(rslt <=0){
      printf("SetESFLA error %s\n", msg);
      return rslt;
   }
   return rslt;
}

//*get esf la***************************************************************************************************/
static int GetESFLA(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_esfla cfg_esfla;

   cfg_esfla.crc = ubxCrc((unsigned char*)&cfg_esfla+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_esfla) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_ESFLA crc %04x\n"    , cfg_esfla.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_esfla;
   // for(i=0;i<sizeof(t_poll_cfg_esfla);i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_esfla, sizeof(t_poll_cfg_esfla), msg);
   if(rslt <=0){
      printf("GetESFLA error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_esfla.Class || frame[3] != cfg_esfla.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printESFLA(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}

/*print esf wt***************************************************************************************************/
static void printESFWT(unsigned char* pdata, unsigned int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char [PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   t_set_get_cfg_esfwt *cfg_esfwt= (t_set_get_cfg_esfwt*)ubxFrame;
   t_str_cfg_esfwt str_cfg_esfwt;
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.version, 		cfg_esfwt->version);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_esfwt.combineTicks, 	cfg_esfwt->flags.combineTicks?"Y":"N");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_esfwt.useWtSpeed, 	cfg_esfwt->flags.useWtSpeed?"Y":"N");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_esfwt.dirPinPol,		cfg_esfwt->flags.dirPinPol?"Y":"N");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %s\n", str_cfg_esfwt.useWtPin, 		cfg_esfwt->flags.useWtPin?"Y":"N");
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.wtFactor, 		cfg_esfwt->wtFactor);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.wtQuantError, 	cfg_esfwt->wtQuantError);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.wtCountMax, 	cfg_esfwt->wtCountMax);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.wtLatency, 	cfg_esfwt->wtLatency);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.wtFrequency, 	cfg_esfwt->wtFrequency);
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%s %d\n", str_cfg_esfwt.speedDeadBand, 	cfg_esfwt->speedDeadBand);
   SettingPrint("ESF WT<06,82>", "%s", printbuff);
   delete[] printbuff;
}

/*set esf wt***************************************************************************************************/
static int SetESFWT(serial_t *pHandle, unsigned char* buff)
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_get_cfg_esfwt *cfg_esfwt = (t_set_get_cfg_esfwt*)buff;
   
   cfg_esfwt->flags.combineTicks = 0;
   cfg_esfwt->flags.useWtSpeed = 1;
   cfg_esfwt->flags.dirPinPol = 0;
   cfg_esfwt->flags.useWtPin = 0;
   cfg_esfwt->crc = ubxCrc((unsigned char*)cfg_esfwt+UBX_CLASS_OFFSET, sizeof(t_set_get_cfg_esfwt)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)cfg_esfwt, sizeof(t_set_get_cfg_esfwt), msg);
   if(rslt <=0){
      printf("SetESFWT error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*get esf wt***************************************************************************************************/
static int GetESFWT(serial_t *pHandle, unsigned char* frame)
{
   int rslt, frameMaxLen = 0, frameLen = 0, num, i, j;
   bool frameStart=false;
   char msg[256];
   unsigned char buff[1024];
   t_poll_cfg_esfwt cfg_esfwt;

   cfg_esfwt.crc = ubxCrc((unsigned char*)&cfg_esfwt+UBX_CLASS_OFFSET, sizeof(t_poll_cfg_esfwt) - UBX_EXCLUDE_CRC_BYTES);

   // printf("POLL CFG_ESFWT crc %04x\n"    , cfg_esfwt.crc);
   // unsigned char* pdata = (unsigned char *)&cfg_esfwt;
   // for(i=0;i<sizeof(t_poll_cfg_esfwt);i++){
   //    printf("%02x ", *pdata++);
   // }
   // printf("\n");

   rslt = writeserial(pHandle, (unsigned char*)&cfg_esfwt, sizeof(t_poll_cfg_esfwt), msg);
   if(rslt <=0){
      printf("GetESFWT error\n");
      return rslt;
   }


   while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }
      for(j=0;j<num;j++){
                                                                         
         if(frameStart==false && buff[j]==UBX_SYNCCHAR_ONE && buff[j+1]==UBX_SYNCCHAR_TWO){
            frameStart = true;
            frameLen = 0;
            frame[frameLen++] = buff[j];
            continue;
         }
         if(frameStart == true){
            frame[frameLen++] = buff[j];
            if(frameLen==4 && (frame[2] != cfg_esfwt.Class || frame[3] != cfg_esfwt.ID)){
               frameStart = false;
               continue;
            }
            if(frameLen==6) frameMaxLen = (int)((frame[UBX_LENGTH_OFFSET+1]<<8)|frame[UBX_LENGTH_OFFSET]);
         }
   
         if(frameLen - frameMaxLen == 8){
            for(i=0;i<frameLen;i++){
               printf("%02x ", frame[i]);
            }
            printf(" Len=%d\n", frameLen);
            printESFWT(frame, frameLen);
            return frameLen;
         }

      }
   }

   return -1;
}

/*set esf reset alg***************************************************************************************************/
static int RESET_ESFALG(serial_t *pHandle)
{
   int rslt;
   unsigned short crc;
   char msg[256];
   t_set_esf_resetalg esf_resetalg;
   
   esf_resetalg.crc = ubxCrc((unsigned char*)&esf_resetalg+UBX_CLASS_OFFSET, sizeof(t_set_esf_resetalg)-UBX_EXCLUDE_CRC_BYTES);

   rslt = writeserial(pHandle, (unsigned char*)&esf_resetalg, sizeof(t_set_esf_resetalg), msg);
   if(rslt <=0){
      printf("Set ESF Reset ALG error %s\n", msg);
      return rslt;
   }
   return rslt;
}

/*print UBX************************************************************************************************/
static void printUBX(unsigned char *pdata, int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);
   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   printbuff = new char [PRINT_BUF_MAX_SIZE];

   for(int k=0;k<ubxFrameMaxLen;k++){
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%02x ", ubxFrame[k]);
     if(k%16 == 15) offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
   }
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");

   if(ubxFrame[3]==POLL_ID15)
     INSPrint("ESF INS<10,15> ", "%s", printbuff);             
   if(ubxFrame[3]==POLL_ID05)
     ATTPrint("NAV ATT<01,05> ", "%s", printbuff);             
   if(ubxFrame[3]==POLL_ID03)
     RAWPrint("ESF RAW<10,03> ", "%s", printbuff);               
   if(ubxFrame[3]==POLL_ID02)
     MEASPrint("ESF MEAS<10,02> ", "%s", printbuff);               
   if(ubxFrame[3]==POLL_ID14)
     ALGPrint("ESF ALG<10,14> ", "%s", printbuff);               
   if(ubxFrame[3]==POLL_ID10)
     STATUSPrint("ESF STATUS<10,10> ", "%s", printbuff);               
   if(ubxFrame[3]==POLL_ID37)
     HNRPrint("NAV HNR<01,37> ", "%s", printbuff);               

   delete[] printbuff;
}

static void ParseAndPrintUBX(unsigned char *pdata, int size)
{
   int offset;
   char *printbuff;
   struct timeval tv;
   unsigned char *ubxFrame = pdata;
   int ubxFrameMaxLen = size;
   gettimeofday(&tv, NULL);

   printbuff = new char[PRINT_BUF_MAX_SIZE];

   offset = snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);

   for(int k=0;k<ubxFrameMaxLen;k++){
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "%02x ", ubxFrame[k]);
     if(k%16 == 15) offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
   }
   offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");

   if(ubxFrame[3]==POLL_ID15){
     offset = 0;
     t_str_esf_ins str_esf_ins;
     t_periodic_polled_esf_ins *esf_ins;
     offset += snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.version,              esf_ins->bitfield0.version);
     if(esf_ins->bitfield0.xAngRateValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.xAngRate,          esf_ins->xAngRate);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.xAngRateValid,              "N");

     if(esf_ins->bitfield0.yAngRateValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.yAngRate,          esf_ins->yAngRate);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.yAngRateValid,              "N");

     if(esf_ins->bitfield0.zAngRateValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.zAngRate,          esf_ins->zAngRate);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.zAngRateValid,              "N");

     if(esf_ins->bitfield0.xAccelValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.xAccelValid,          esf_ins->xAccel);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.xAccelValid,              "N");

     if(esf_ins->bitfield0.yAccelValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.yAccelValid,          esf_ins->yAccel);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.yAccelValid,              "N");

     if(esf_ins->bitfield0.zAccelValid)
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_ins.zAccelValid,          esf_ins->zAccel);
     else
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_ins.zAccelValid,              "N");

     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
     INSPrint("ESF INS<10,15> ", "%s", printbuff);
   }
   if(ubxFrame[3]==POLL_ID05){
     offset = 0;
     t_str_nav_att str_nav_att;
     t_periodic_polled_nav_att *nav_att;
     offset += snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.iTOW,              nav_att->iTOW);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.version,           nav_att->version);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.roll,              nav_att->roll);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.pitch,             nav_att->pitch);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.heading,           nav_att->heading);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.accRoll,           nav_att->accRoll);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.accPitch,          nav_att->accPitch);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_att.accHeading,        nav_att->accHeading);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
     ATTPrint("NAV ATT<01,05> ", "%s", printbuff);
   }
   if(ubxFrame[3]==POLL_ID03)
     RAWPrint("ESF RAW<10,03> ", "%s", printbuff);             
   if(ubxFrame[3]==POLL_ID02)
     MEASPrint("ESF MEAS<10,02> ", "%s", printbuff);
   if(ubxFrame[3]==POLL_ID14){
     offset = 0;
     t_str_esf_alg str_esf_alg;
     t_periodic_polled_esf_alg *esf_alg = (t_periodic_polled_esf_alg*) ubxFrame;
     offset += snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_alg.iTOW,              esf_alg->iTOW);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_alg.version,           esf_alg->version);
     (esf_alg->bitfield0.status<(sizeof(str_esf_alg.status)/sizeof(str_esf_alg.status[0])))?												\
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_alg.status_tag,        str_esf_alg.status[esf_alg->bitfield0.status]):			\
     offset += 0;
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_alg.error,  		esf_alg->bitfield0.error?"E":"N");
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf\n", str_esf_alg.roll,		esf_alg->roll*1e-1);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf\n", str_esf_alg.pitch,		esf_alg->pitch*1e-1);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf\n", str_esf_alg.yaw,		esf_alg->yaw*1e-1);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
     ALGPrint("ESF ALG<10,14> ", "%s", printbuff);
   }
   if(ubxFrame[3]==POLL_ID10){
     offset = 0;
     t_str_esf_status str_esf_status;
     t_periodic_polled_esf_status *esf_status = (t_periodic_polled_esf_status*) ubxFrame;
     offset += snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_status.iTOW,              esf_status->iTOW);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_status.version,           esf_status->version);
     (esf_status->initStatus.wtInitStatus<(sizeof(str_esf_status.wtInitStatus)/sizeof(str_esf_status.wtInitStatus[0])))?								\
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_status.initStatus_wtInit, str_esf_status.wtInitStatus[esf_status->initStatus.wtInitStatus]):	\
     offset += 0;
     (esf_status->initStatus.mntAlgStatus<(sizeof(str_esf_status.mntAlgStatus)/sizeof(str_esf_status.mntAlgStatus[0])))?								\
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_status.initStatus_mntAlg, str_esf_status.mntAlgStatus[esf_status->initStatus.mntAlgStatus]):	\
     offset += 0;
     (esf_status->status<(sizeof(str_esf_status.status)/sizeof(str_esf_status.status[0])))?												\
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %s\n", str_esf_status.status_tag,        str_esf_status.status[esf_status->status]):				\
     offset += 0;
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_esf_status.numSens,           esf_status->numSens);
     for(int i=0;i<esf_status->numSens;i++){
	offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t-----------------------------------------------------------------------------\n");
        t_esf_status_sens* sens=&esf_status->sens + i;
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %d\n", str_esf_status.sensStatus1_type,  sens->sensStatus1.type);
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.sensStatus1_used,  sens->sensStatus1.used?"Y":"N");
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.sensStatus2_calib, str_esf_status.calibStatus[sens->sensStatus2.calibStatus]);
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.sensStatus2_time,  str_esf_status.timeStatus[sens->sensStatus2.timeStatus]);
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.faults_badMeas,        	sens->faults.badMeas?"Y":"N");
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.faults_badTTag,        	sens->faults.badTTag?"Y":"N");
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.faults_missingMeas,	sens->faults.missingMeas?"Y":"N");
        offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t\t%s %s\n", str_esf_status.faults_noisyMeas,      	sens->faults.noisyMeas?"Y":"N");
     }
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
     STATUSPrint("ESF STATUS<10,10> ", "%s", printbuff);               
   }
   if(ubxFrame[3]==POLL_ID37){
     offset = 0;
     t_str_nav_hnr str_nav_hnr;
     t_periodic_polled_nav_hnr *nav_hnr = (t_periodic_polled_nav_hnr*) ubxFrame;
     offset += snprintf(printbuff, PRINT_BUF_MAX_SIZE, "class %02x id %02x, length %d, %ld.%ld\n", ubxFrame[2], ubxFrame[3], ubxFrameMaxLen, tv.tv_sec, tv.tv_usec);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %d\n", str_nav_hnr.TOW, nav_hnr->TOW);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%d/%d/%d %d-%d-%d.%d\n", 
                              nav_hnr->year, 
                              nav_hnr->month, 
                              nav_hnr->day, 
                              nav_hnr->hour, 
                              nav_hnr->min, 
                              nav_hnr->sec,
                              nav_hnr->nano);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s(%s) %s(%s) %s(%s)\n", 
                              str_nav_hnr.valids[0], nav_hnr->valid.validDate?"V":"I", 
                              str_nav_hnr.valids[1], nav_hnr->valid.validTime?"V":"I",
                              str_nav_hnr.valids[2], nav_hnr->valid.fullyResolved&0x01?"V":"I");
     (nav_hnr->gpsFix<(sizeof(str_nav_hnr.fixStatus)/sizeof(str_nav_hnr.fixStatus[0])))?												\
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\tgpsFix %s\n", str_nav_hnr.fixStatus[nav_hnr->gpsFix]):								\
     offset += 0;
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\tflag %s(%s) %s(%s) %s(%s) %s(%s) %s(%s)\n",
                              str_nav_hnr.flags[0], nav_hnr->flags.GPSfixOK?"V":"I",
                              str_nav_hnr.flags[1], nav_hnr->flags.DiffSoln?"V":"I",
                              str_nav_hnr.flags[2], nav_hnr->flags.WKNSET?"V":"I",
                              str_nav_hnr.flags[3], nav_hnr->flags.TOWSET?"V":"I",
                              str_nav_hnr.flags[4], nav_hnr->flags.headVehValid?"V":"I");
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf, %s %lf\n", str_nav_hnr.latitude, nav_hnr->lat*1e-7, str_nav_hnr.longitude, nav_hnr->lon*1e-7);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf, %s %lf\n", str_nav_hnr.height, nav_hnr->height*1e-2, str_nav_hnr.hMSL, nav_hnr->hMSL*1e-2);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf, %s %lf\n", str_nav_hnr.gspeed, nav_hnr->gSpeed*1e-2, str_nav_hnr.speed, nav_hnr->speed*1e-2);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\t%s %lf, %s %lf\n", str_nav_hnr.headMot, nav_hnr->headMot*1e-5, str_nav_hnr.headVeh, nav_hnr->headVeh*1e-5);
     offset += snprintf(printbuff+offset, PRINT_BUF_MAX_SIZE-offset, "\n");
     HNRPrint("NAV HNR<01,37> ", "%s", printbuff);               
   }

   delete[] printbuff;
}

void* ReceiveGPSMessage(void *arg)
{
    int num;
    char msg[256];
    unsigned char buff[1024];
    unsigned char frame[4096];
    unsigned char ubxFrame[4096];
    int i, frameLen, ubxFrameLen, ubxFrameMaxLen;
    int offset=3;
    bool frameStart=false;
    bool ubxFrameStart=false;
    unsigned char rate[6] = {0,2,0,0,0,0};
 
    // "ttyUSB0:9600:8:n:1:off"
    pHandle = openserial((const char*)arg, STR_MODE_RW, msg);
    if(pHandle == NULL){
       printf("open the serial error [%s]\n", msg);
       return NULL;
    }
 
    // set the ESFWT to use the speed measurements
    num = GetESFWT(pHandle, frame);
    if(num > 0){
       num = SetESFWT(pHandle, frame);
       if(num > 0){
          num = GetESFWT(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-ESFWT\n");
          }
       }
    }
 
    unsigned int antenna[3]={0,0,100};
    unsigned int accel[3]={0,0,0};
    // set the ESFLA to use the speed measurements
    num = GetESFLA(pHandle, frame);
    if(num > 0){
       num = SetESFLA(pHandle, frame, antenna, accel);
       if(num > 0){
          num = GetESFLA(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-ESFLA\n");
          }
       }
    }
 
    bool enable=true;
    signed int angle[3]={0,0,0};
    // set the ESFALG to use the speed measurements
    num = GetESFALG(pHandle, frame);
    if(num > 0){
       num = SetESFALG(pHandle, frame, enable, angle);
       if(num > 0){
          num = GetESFALG(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-ESFALG\n");
          }
       }
    }
 
    bool enableAdr=true;
    // set the cfg navx5 to enable adr
    num = Get_CFG_NAVX5(pHandle, frame);
    if(num > 0){
       num = Set_CFG_NAVX5(pHandle, frame, enableAdr);
       if(num > 0){
          num = Get_CFG_NAVX5(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-NAVX5\n");
          }
       }
    }
 
    unsigned int navRate=500;
    // set the cfg rate to enable adr
    num = Get_CFG_Rate(pHandle, frame);
    if(num > 0){
       num = Set_CFG_Rate(pHandle, frame, navRate);
       if(num > 0){
          num = Get_CFG_Rate(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-Rate\n");
          }
       }
    }

    unsigned int highNavRate=40; // highNavRate is half actually, 40 for 20HZ;
    // set the cfg HNR to enable adr
    num = Get_CFG_HNR(pHandle, frame);
    if(num > 0){
       num = Set_CFG_HNR(pHandle, frame, highNavRate);
       if(num > 0){
          num = Get_CFG_HNR(pHandle, frame);
          if(num > 0){
             printf("Finish to Set the CFG-HNR\n");
          }
       }
    }

    // reset the alg
    // RESET_ESFALG();

    // set the CFGMSG to get the Sensor Fuision Data
    if(strcmp(getenv(STR_DEBUG_ESF_MEAS) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID02);
       if(0 == rate[1]){
          rate[1] = 2;
          (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID02);
       }
    }

    // set the CFGMSG to get the Sensor Fuision Data
    if(strcmp(getenv(STR_DEBUG_ESF_RAW) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID03);
       if(0 == rate[1]){
          rate[1] = 2;
          (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID03);
       }
    }

    // set the CFGMSG to get the Sensor Mount Alignment Information
    if(strcmp(getenv(STR_DEBUG_ESF_ALG) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID14);
       if(0 == rate[1]){
         rate[1] = 2;
         (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID14);
       }
    }

    // set the CFGMSG to get the Sensor fusion status information
    if(strcmp(getenv(STR_DEBUG_ESF_STATUS) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID10);
       if(0 == rate[1]){
         rate[1] = 2;
         (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID10);
       }
    }

   // set the CFGMSG to get ESF INS
    if(strcmp(getenv(STR_DEBUG_ESF_INS) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID15);
       if(0 == rate[1]){
         rate[1] = 2;
         (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS10, POLL_ID15);
       }
    }

    // set the CFGMSG to get NAV HNR
    if(strcmp(getenv(STR_DEBUG_NAV_HNR) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS01, POLL_ID37);
       if(0 == rate[1]){
         rate[1] = 2;
         (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS01, POLL_ID37);
       }
    }

    // set the CFGMSG to get NAV ATT
    if(strcmp(getenv(STR_DEBUG_NAV_ATT) , "on")==0){
       (void)GetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS01, POLL_ID05);
       if(0 == rate[1]){
         rate[1] = 2;
         (void)SetCFGMSG(pHandle, rate, sizeof(rate), POLL_CLASS01, POLL_ID05);
       }
    }


    while(!MainExit){
      num = readserial(pHandle, buff, sizeof(buff), msg);
      if(num == 0){
         // printf("read the serial error [%s]\n", msg);
         continue;
      }

      for(int i=0;i<num;i++)
      {
          // Parse the ubx protocol
          if(ubxFrameStart == false && buff[i] == UBX_SYNCCHAR_ONE){
              ubxFrameStart = true;
              ubxFrameLen = 0;
              ubxFrameMaxLen = -1;
          }

          if(ubxFrameStart == true){
	     ubxFrame[ubxFrameLen++]=buff[i];
             if(ubxFrameLen == 2 && buff[i] != UBX_SYNCCHAR_TWO) ubxFrameStart = false;
             if(ubxFrameLen == 3 && buff[i] != POLL_CLASS10 && buff[i] != POLL_CLASS01) ubxFrameStart = false;
             if(ubxFrameLen == 4 && 
                buff[i] != POLL_ID02 && 
                buff[i] != POLL_ID03 && 
                buff[i] != POLL_ID05 && 
                buff[i] != POLL_ID14 && 
                buff[i] != POLL_ID15 && 
                buff[i] != POLL_ID10 && 
                buff[i] != POLL_ID37) 
                   ubxFrameStart = false;
             if(ubxFrameLen == 6) ubxFrameMaxLen = (ubxFrame[4]|(ubxFrame[5]<<8))+8;
             
             if(ubxFrameLen == ubxFrameMaxLen){
                ParseAndPrintUBX(ubxFrame, ubxFrameMaxLen);
                // printUBX(ubxFrame, ubxFrameMaxLen);
                memset(ubxFrame, 0, sizeof(ubxFrame));
                ubxFrameStart = false;
             }
          }

          // Parse the NMEA0183 protocol
          if(frameStart == false && buff[i] == '$'){
             frameStart = true;
             frameLen=0;
          }

          if(frameStart == true){
   
    	     if(buff[i]=='\r')
                continue;
             if(buff[i]!='\n')
                 frame[frameLen++] = buff[i];
             else{
                 // printf("%d: %s\n", (int)strlen(TAG_GNRMC), frame);
                 if(strncmp((const char*)frame+offset, TAG_GNRMC, strlen(TAG_GNRMC) ) == 0)
                     GNRMC((char*)frame , strlen((const char*)frame));
                 else if(strncmp((const char*)frame+offset, TAG_GNGLL, strlen(TAG_GNGLL) ) == 0)
                     GNGLL((char*)frame , strlen((const char*)frame));
                 else if(strncmp((const char*)frame+offset, TAG_GNGGA, strlen(TAG_GNGGA) ) == 0)
                     GNGGA((char*)frame , strlen((const char*)frame));
                 else if(strncmp((const char*)frame+offset, TAG_GNVTG, strlen(TAG_GNVTG) ) == 0)
                     GNVTG((char*)frame , strlen((const char*)frame));
                 else if(strncmp((const char*)frame+offset, TAG_GNGSA, strlen(TAG_GNGSA) ) == 0)
                     GNGSA((char*)frame , strlen((const char*)frame));
                 else if(strncmp((const char*)frame+offset, TAG_XXGSV, strlen(TAG_XXGSV) ) == 0)
                     GXGSV((char*)frame , strlen((const char*)frame));
                 memset(frame, 0, sizeof(frame));
                 frameStart = false;
             }
          }
    

      }
   }

Exit:
   closeserial(pHandle);
   pHandle = NULL;
   return NULL;
}

int main()
{
   int rslt;
   const char* path;
   pthread_t canPthread;
   pthread_t gpsPthread;

   setenv(STR_DEBUG_NMEA, 	"off", 0);
   setenv(STR_DEBUG_MEASPEED, 	"off", 0);
   setenv(STR_DEBUG_POS, 	"off", 0);
   setenv(STR_DEBUG_CAN, 	"off", 0);
   setenv(STR_DEBUG_ESF_MEAS, 	"off", 0);
   setenv(STR_DEBUG_ESF_RAW, 	"off", 0);
   setenv(STR_DEBUG_ESF_ALG, 	"off", 0);
   setenv(STR_DEBUG_ESF_STATUS, "off", 0);
   setenv(STR_DEBUG_NAV_HNR,	"off", 0);
   setenv(STR_DEBUG_SETTING,	"off", 0);
   setenv(STR_DEBUG_ESF_INS,	"off", 1);
   setenv(STR_DEBUG_NAV_ATT,	"off", 1);

   printf("Debug NMEA =%s=\n",			getenv(STR_DEBUG_NMEA));
   printf("Debug Measurement Speed  =%s=\n", 	getenv(STR_DEBUG_MEASPEED));
   printf("Debug POS =%s=\n",			getenv(STR_DEBUG_POS));
   printf("Debug CAN =%s=\n",			getenv(STR_DEBUG_CAN));
   printf("Debug ESF MEAS =%s=\n",		getenv(STR_DEBUG_ESF_MEAS));
   printf("Debug ESF RAW =%s=\n",		getenv(STR_DEBUG_ESF_RAW));
   printf("Debug ESF ALG =%s=\n",		getenv(STR_DEBUG_ESF_ALG));
   printf("Debug ESF STATUS =%s=\n",		getenv(STR_DEBUG_ESF_STATUS));
   printf("Debug NAV HNR =%s=\n",		getenv(STR_DEBUG_NAV_HNR));
   printf("Debug SETTING =%s=\n",		getenv(STR_DEBUG_SETTING));
   printf("Debug ESF INS =%s=\n",		getenv(STR_DEBUG_ESF_INS));
   printf("Debug NAV ATT =%s=\n",		getenv(STR_DEBUG_NAV_ATT));

   if(access("/dev/" UARTINTERFACE0, R_OK) == 0)
      path = UARTINTERFACE0 ":230400:8:n:1:off";
   else
      path = UARTINTERFACE1 ":230400:8:n:1:off";

   signal(SIGINT,  ShellSignalHandle);
   signal(SIGTERM, ShellSignalHandle);
   signal(SIGQUIT, ShellSignalHandle);
   signal(SIGHUP,  ShellSignalHandle);

   rslt = pthread_create(&canPthread, NULL, &ReceiveGPSMessage, (void*)path);
   sleep(2);
   rslt = pthread_create(&canPthread, NULL, &ReceiveCANMessagewithPCAN, (void*)pHandle);

   // StartTimer(100*1000, ConfigureSignalRoutine);
   while(!MainExit){
      sleep(10);
   }
   return 0;
}

