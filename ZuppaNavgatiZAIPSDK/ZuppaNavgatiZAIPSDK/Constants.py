#---------Python 3.8.8
#Author : Venkatesh Sai
#Ts     : 12:05p.m / 04-04-2022
#© Copyrighted (2021) : Venkatesh Sai , Zuppa Geo Navigation Technologies Pvt. Ltd . Github Repo : f434a20243df049cc079d2ab2e5cf9b24a0170df
import datetime
import enum  
import numpy as np
import struct
import serial


#----------WIRELESS DEFINES----------------
FWD_PORT_TIMEOUT=6000
TELEM_LOST_TIMEOUT=7000
HEATBEAT_PACKET=4500
TARGET_POSITION_PACKET=4500
MSG_FORWARD_TIMEOUT=6000 # 6 seconds to enabled forwarding
SERIAL_LIB_FLIR=serial
DEFAULT_BAUDRATE=9600
SDK_ANGLE_SCALE_FACTOR=1.75929
SDK_UPDATE_TIME=100


#---------------------------------------
'''
 (controlParams.pcControlEnable & 0x40)?asopAssigned=1:1;
	(controlParams.pcControlEnable & 0x20)?assignRoll=1:1;
	(controlParams.pcControlEnable & 0x10)?assignPitch=1:1;
	(controlParams.pcControlEnable & 0x08)?assignPwr=1:1;
	(controlParams.pcControlEnable & 0x04)?assignHeading=1:1;
	(controlParams.pcControlEnable & 0x02)?assignArm=1:1;
'''

class PC_CONTROL(enum.IntEnum):
    ASSIGN_ASOP=0x40
    ASSIGN_ROLL=0x20
    ASSIGN_PITCH=0x10
    ASSIGN_POWER=0x08
    ASSIGN_HDG=0x04
    ASSIGN_ARM=0x02

class AERODYNAMIC_LIMITS(enum.IntEnum): # 75 degreees
    MAX_ROLL_ANGLE=30   
    MAX_PITCH_ANGLE=30  # 30 degrees MAX PITCH ANGLE
    MAX_YAW_INPUT=20    # 20 degrees per second
    MAX_POWER_PWM=2000  # 2000 PwM Divisions
    


class WL_TERMINATION_TASK(enum.IntEnum):
    HOME_WP=0
    CURRENT_WP=1
    TARGET_WP=2
    REFERENCE_WP=3
    TAKEOFF_WP=4
    LANDING_WP=5
    APPROCH_WP=6
    GENERAL_WP=7
    SPECIAL_WP=8
    HOLD_WP=9
    CIRCLE_WP=10
    RTH_WP=11
    INVALID=12

class WL_LOCATION_POINTER(enum.IntEnum):
    LOC_PNTR_NULL=0
    LOC_PNTR_CURRENT=1
    LOC_PNTR_HOME=2
    LOC_PNTR_TARGET=3
    LOC_PNTR_WAYPOINT=4

class WL_CALIBRATION(enum.IntEnum):
    NO_CALIBRATION=0
    IMU_CALIBRATION=1
    MAG_CALIBRATION=2
    ESC_CALIBRATION=3
    RCV_CALIBRATION=4


class WL_MEMORY_WRITE(enum.IntEnum):
    MEM_WRITE_PID_CORE1=0xF0
    MEM_WRITE_PID_CORE2=0x08
    MEM_WRITE_WPS_CORE2=0x04
    MEM_WRITE_PAR_CORE2=0x02
    MEM_NO_COMMAND=0x00


class WL_PROTOCOL_ATTRIBUTES(enum.IntEnum):
    HEADER1=0xB5
    HEADER2=0x62
    INVALID_CMD=0x00
    INVALID_DATA=0xFF

class WL_PACKETS(enum.IntEnum):
    PKT_KINEMATIC=0x00
    PKT_CAL      =0x01
    PKT_PID      =0x02
    PKT_POINT    =0x03
    PKT_GPCG     =0x04
    PKT_CPP      =0x05
    PKT_COP_PKT  =0x06
    PKT_CNTRL_PKT=0x07
    PKT_PIDOP_PKT=0x08
    PKT_AUX1_PKT =0x09
    PKT_AUX1_CONF=0x0A

WL_PAC_LABELS=[
    "PKT_KINEMATIC",
    "PKT_CAL",
    "PKT_PID",
    "PKT_POINT",
    "PKT_GPCG",
    "PKT_CPP",
    "PKT_COP_PKT",
    "PKT_CNTRL_PKT",
    "PKT_PIDOP_PKT",
    "PKT_AUX1_PKT",
    "PKT_AUX1_CONF"
    ]

class WL_PACKETS_PL(enum.IntEnum):
    KINEMATIC_PL=23
    CAL_PL=2
    PID_PL=13
    PID_GET_PL=1
    POINT_PL=12
    POINT_GET_PL=1
    POINT_SET_MAX_PL=2
    GPCG_PL=14
    CPP_PL=14
    CPP_GET_PL=1
    COP_PL=2
    CNTRL_PL=10
    CNTRL_SET_PL=1
    PIDOP_PL=14
    PIXYPL=14
    AUX1_PL=11
    AUX1_CONF_PL=20
    MAX_PACKET_LENGTH=23

class WL_ERRORS(enum.IntEnum):
    TELEM_LOST =0x00
    MAG_DISCONN=0x01

class FLIGHT_MODES(enum.IntEnum):
    ARMED=0
    DISARMED=1
    FLYING=2
    SAFETY_CHECK=3
    MANUAL=4
    LOOP=5
    TAKEOFF=6 # takeoff -> 120 meters @ 60 kmph speed (Radio Throttle stick to be center position) -> RTH -> LOITER (Recevier command should 1500)
    LANDING=7
    EZ_FLY=8
    BARO_HOLD=9
    POSITION_HOLD=10
    #---------AUTO_MODES-----------------
    NAVIGATION=11
    RTH=12
    AUTO_CIRCLE=13
    LOITER=14
    NO_TASK=15

class PID_POINTERS(enum.IntEnum):
    RATE_XAXIS_PID_IDX=0
    RATE_YAXIS_PID_IDX=1
    ZAXIS_PID_IDX=2
    BARO_RATE_PID_IDX=3
    ATTITUDE_XAXIS_PID_IDX=4
    ATTITUDE_YAXIS_PID_IDX=5
    HEADING_HOLD_PID_IDX=6
    BARO_PID_IDX=7
    SPEED_X_PID_IDX=8
    SPEED_Y_PID_IDX=9
    MAX_NO_OF_PID_PACKETS=10

#----------WIRELESS VARIABLES------------
class Position:

    def __init__(self):
        self.latitude=0
        self.longitude=0
        self.altitude=0
        self.type=0

class Attitude:

    def __init__(self):
        self.rollDeg=0
        self.pitchDeg=0
        self.yawDeg=0
        self.gyroRoll=0
        self.gyroPitch=0
        self.gyroYaw=0
        self.pSampleTime=0

class PIDPacket:

    def __init__(self):
        self.P=0
        self.I=0
        self.D=0
        self.type=0

class InternalSystemVars:

    def __init__(self):
        self.outputPIDPower=0
        self.gpsCorrXDegs=0
        self.gpsCorrYDegs=0
        self.rollSpdDes=0
        self.pitchSpdDes=0
        self.pitchIntErr=0
        self.rollIntErr=0

class AdvancedSystemVars:

    def __init__(self):
        self.status=0
        self.baroRate=0
        self.lidarHt=0
        self.magX=0
        self.magY=0
        self.magZ=0

class VehicleControlObject:

    def validateParams(self):
        return True

    def assignControl(self,rollAngDeg,pitchAngDeg,yawDPS,power):
        self.angleX=rollAngDeg;
        self.angleY=pitchAngDeg;
        self.yaw=yawDPS;
        self.power=power;
        if(self.validateParams()):
            self.takeControl=1;
            self.assignCmd=1;

    def toggleArm(self):
        self.armSendCnts=10;

    def withDrawControl(self):
        self.takeControl=0;

    def __init__(self):
        self.armSendCnts=0;
        self.sdkInControl=0;
        self.takeControl=0;
        self.assignCmd=0;
        self.angleX=0;
        self.angleY=0;
        self.yaw=0;
        self.power=1000;

 # MAIN READ AND WRITE OBJECT CLASS
class VehicleParams:

    def setFlightMode(self,mode):
        ret=False;
        if(mode<FLIGHT_MODES.LOITER):
            self.assignMode=mode;
            self.setMode=1;
            ret=True;
        return ret;

    def setTargetPoint(self,latFull,lonFull,altMeters):
        if(self.assignTargetPoint<=0):
            self.targetLat=latFull;
            self.targetLon=lonFull;
            self.targetAlt=altMeters;
            self.assignTargetPoint=3;

    def initParams(self,maxRange=3000,maxAltitude=300): # maxRange in meters , maxaltitude in meters , this is to limit the input data
        #-------------------
        self.assignTargetPoint=0;
        self.targetPointValid=0;
        self.targetAlt=0;
        self.targetLat=0;
        self.targetLon=0;
        self.targetWaypointNumber=0
        self.assignMode=FLIGHT_MODES.MANUAL;
        self.setMode=0;
        #--------------------

        self.vehicleControl=VehicleControlObject(); # Main Object Command & Control
        self.advancedParams=AdvancedSystemVars()
        self.internalParams=InternalSystemVars()
        self.wayPointList=[Position()]*96 # Max No of waypoints
        self.pidData=[PIDPacket()]*PID_POINTERS.MAX_NO_OF_PID_PACKETS
        self.declination=0
        self.calibType=0
        self.memType=0
        self.sdkInControl=False
        self.gotAuxPacket=0
        self.pDataTime=0
        self.packetCounter=0
        self.connectedToAutopilot=False
        self.errorOccured=0
        self.currentPosition=Position()
        self.homePosition=Position()
        self.targetPosition=Position()
        self.altitudeAGLcm=0
        self.altitudeMSLcm=0
        self.heading=0
        self.gpsCourse=0
        self.speedXCms=0
        self.speedYCms=0
        self.speedCmsGps=0
        self.speedCmsINS=0
        self.attitude=Attitude()
        self.varioRateCms=0
        self.speed3DIns=0
        self.tripMeterMeters=0
        self.gpsAccuracyMeters=0
        self.maxNoOfSats=0
        self.angleToTgtDeg=0
        self.distanceToHomeMeters=0
        self.distanceToTargetMeters=0
        self.batteryVoltage=0
        self.dataTimeStamp="00:00:00"
        self.currentFlightMode=FLIGHT_MODES.MANUAL
        self.armedStatus=FLIGHT_MODES.DISARMED
        self.gpsLocked=False
        self.gpsGotHomePoint=False
        self.insRelocated=0
        self.rollCommand=0
        self.pitchCommand=0
        self.powerCommand=0
    
    def __init__(self):
        self.initParams()


#----------GLOBAL DEFINES----------------
buff=['H','e','L','L','@']
def getIntegerValue(len,buffer):
    ret=0
    pointer=0
    for i in range(len):
        ret|=(ord(buffer[pointer])<<(i*8))
        pointer+=1
    return ret

print(getIntegerValue(2,buff))

def getFloatValue(buffer):
    ret=0
    pointer=0;
    arr=bytearray()
    for i in range(4):
        val=ord(buffer[pointer])
        pointer+=1
        arr.extend(np.byte(val))
    ret=struct.unpack('f',arr)

    return ret
#0x42596148
buffer=[chr(0x48),chr(0x61),chr(0x59),chr(0x42)]
print(getFloatValue(buffer))

def setIntegerValue(val,len):
    buffer=[]
    pointer=0
    for i in range(len):
        buffer.append(chr(np.ubyte((val & (0xFF << (i*8)))>>(i*8))))
        pointer+=1
    return buffer
    

val=getIntegerValue(5,buff)
chk=setIntegerValue(val,5)
print(val,str(chk),chk)
#----------GLOBAL VARIABLES--------------

#----------GLOBAL FUNCTIONS--------------
#--------FUNCTIONS--------
def millis():
        ct = datetime.datetime.now()
        return (ct.timestamp()*1000)

def getTimestamp():
        ct = datetime.datetime.now()
        return ct


