#---------Python 3.8.8
#Author : Venkatesh Sai
#Ts     : 12:05p.m / 04-04-2022
#© Copyrighted (2021) : Venkatesh Sai , Zuppa Geo Navigation Technologies Pvt. Ltd . Github Repo : f434a20243df049cc079d2ab2e5cf9b24a0170df
import Constants
import time
import ByteStream
import math
class Protocol:

    def incrementCheckSums(self,a):
      #  print(type(a),type(self.chkA))
        self.chkA+=a
        if(self.chkA>255):
            self.chkA=self.chkA-256
        self.chkB+=self.chkA
        if(self.chkB>255):
            self.chkB=self.chkB-256

    def incrementOutputCheckSums(self,a):
        self.opChkA+=a
        if(self.opChkA>255):
            self.opChkA=self.opChkA-256
        self.opChkB+=self.opChkA
        if(self.opChkB>255):
            self.opChkB=self.opChkB-256

    def __init__(self):
        self.sendBuffer=[]
        self.sendLength=0
        self.rcvBuffer=[]
        self.rcvLength=0
        self.commandType=Constants.WL_PROTOCOL_ATTRIBUTES.INVALID_CMD
        self.payloadLength=Constants.WL_PROTOCOL_ATTRIBUTES.INVALID_DATA
        self.numberOfBytesPresent=Constants.WL_PROTOCOL_ATTRIBUTES.INVALID_DATA
        self.payloadCounter=0
        self.step=0
        self.chkA=self.chkB=0
        self.opChkA=0
        self.opChkB=0;
        self.dataArrvied=False
        self.pRcvTime=0
'''
uint8_t modes;
	int16_t x;
	int16_t y;
	int16_t heading;
	uint16_t pwr;
	uint8_t enableControls;
'''
class PacketGenerators:

    def __init__(self):
        self.outputBuffer=[]
        self.pAsop=0;
        self.packetGen=False
        self.outputStream = ByteStream.Stream()

    def generateCalibrationPacket(self,calToDo,writeToMem):
        self.outputBuffer=[]
        self.outputBuffer.append(chr(calToDo))
        self.outputBuffer.append(chr(writeToMem))
        self.packetGen=True

    def generatePacketFetch(self,pntr):
        self.outputBuffer=[]
        self.outputStream.opStream.clearBuffer()
        self.outputStream.opStream.uint8_t(pntr)
        self.outputStream.opStream.int32_t(0)
        self.outputStream.opStream.int32_t(0)
        self.outputStream.opStream.uint16_t(0)
        self.outputStream.opStream.uint8_t(0)
        self.outputBuffer=self.outputStream.opStream.buffer

    def generateTargetLocationPacket(self,lat,lon,alt):
        self.outputBuffer=[]
        self.outputStream.opStream.clearBuffer();
        self.outputStream.opStream.uint8_t(Constants.WL_LOCATION_POINTER.LOC_PNTR_TARGET)
        self.outputStream.opStream.int32_t(lat);
        self.outputStream.opStream.int32_t(lon);
        self.outputStream.opStream.int32_t(alt);
        self.outputStream.opStream.uint8_t(Constants.WL_TERMINATION_TASK.HOLD_WP);
        self.outputBuffer=self.outputStream.opStream.buffer
    '''
   uint8_t modes;
	int16_t x;
	int16_t y;
	int16_t heading;
	uint16_t pwr;
	uint8_t enableControls;

    '''
    def generateCommandPacket(self,mode,roll,pitch,yaw,power,sdkInControl,armToggle):
        chk=0
        rl=0
        pt=0
        if(Constants.DRONE_TYPE==Constants.DRONE_TYPES.MULTIROTOR):
            rl=int((roll/1.75929)*10.00)
            pt=int((pitch/1.75929)*10.00);
        else:
            rl=int(roll*1.75929);
            pt=int(pitch*1.75929);
        control=0;
        self.outputBuffer=[]
        self.outputStream.opStream.clearBuffer();
        chk=(mode <<4) & 0xF0;
        if(sdkInControl==False):
            rl=0;
            pt=0;
            yaw=0;
        self.outputStream.opStream.uint8_t(chk);
        self.outputStream.opStream.int16_t(rl);
        self.outputStream.opStream.int16_t(pt);
        self.outputStream.opStream.int16_t(int(yaw));
        self.outputStream.opStream.uint16_t(int(power));
        #-------CONTROL VAR----------
        if(self.pAsop!=mode):
            control|=Constants.PC_CONTROL.ASSIGN_ASOP;
        self.pAsop=mode;
        if(sdkInControl==True):
            control|=Constants.PC_CONTROL.ASSIGN_ROLL;
            control|=Constants.PC_CONTROL.ASSIGN_PITCH;
            control|=Constants.PC_CONTROL.ASSIGN_POWER;
            control|=Constants.PC_CONTROL.ASSIGN_HDG;
        if(armToggle):
            control|=Constants.PC_CONTROL.ASSIGN_ARM
        self.outputStream.opStream.uint8_t(control);
        self.outputBuffer=self.outputStream.opStream.buffer;
       # print("LeN: ",len(self.outputBuffer))
    

        





class WLMessageParser:

    def __init__(self):
        self.tgtSendCnts=0;
        self.outputBuffer=[]
        self.pTakeControl=0;
        self.protocol=Protocol()
        self.parsedHomePoint=False
        self.pTargetPositionFetchTime=0
        self.pHeatBeatFetch=0
        self.pSDKSendTime=0;
        self.pRl=0
        self.pPt=0
        self.pYw=0
        self.pAlt=0
        self.vehicle=Constants.VehicleParams() # Main Object Declaration
        self.packetGen=PacketGenerators()
        self.inputStream  = ByteStream.Stream()


        

    def parseData(self):
       # print("PNTR:",self.protocol.commandType,Constants.WL_PAC_LABELS[self.protocol.commandType])
        self.inputStream.inStream.clearBuffer()
        self.inputStream.inStream.assignDataBuffer(self.protocol.rcvBuffer)
        if(self.protocol.commandType==Constants.WL_PACKETS.PKT_KINEMATIC):
            self.vehicle.attitude.rollDeg=float(self.inputStream.inStream.int16_t())*(1.8/math.pi)
            self.vehicle.attitude.pitchDeg=float(self.inputStream.inStream.int16_t())*(1.8/math.pi)
            self.vehicle.attitude.yawDeg=float(self.inputStream.inStream.int16_t())*(1.8/math.pi)
            self.vehicle.altitudeAGLcm=self.inputStream.inStream.float()
            if(self.vehicle.attitude.pSampleTime!=0):
                dt=float(Constants.millis()-self.vehicle.attitude.pSampleTime)/1000
                if(dt<=0):
                    dt=0.1
                self.vehicle.varioRateCms=(self.vehicle.altitudeAGLcm-self.pAlt)/dt
                self.vehicle.attitude.gyroRoll=(self.vehicle.attitude.rollDeg-self.pRl)/dt
                self.vehicle.attitude.gyroPitch=(self.vehicle.attitude.pitchDeg-self.pPt)/dt
                self.vehicle.attitude.gyroYaw=(self.vehicle.attitude.yawDeg-self.pYw)/dt
            self.pAlt=self.vehicle.altitudeAGLcm
            self.pRl=self.vehicle.attitude.rollDeg
            self.pPt=self.vehicle.attitude.pitchDeg
            self.pYw=self.vehicle.attitude.yawDeg
            self.vehicle.attitude.pSampleTime=Constants.millis()
            mode=self.inputStream.inStream.uint8_t()
            self.vehicle.armedStatus=(mode & 0x0F)
            self.vehicle.currentFlightMode=((mode & 0xF0)>>4)
            self.vehicle.speedXCms=float(self.inputStream.inStream.int16_t())
            self.vehicle.speedYCms=float(self.inputStream.inStream.int16_t())
            self.vehicle.speedCmsGps=self.inputStream.inStream.uint16_t()
            self.vehicle.batteryVoltage=(self.inputStream.inStream.uint16_t() & 0x7FFF)
            if((self.vehicle.batteryVoltage & 0x8000)>0):
                self.vehicle.sdkInControl=1
                self.vehicle.vehicleControl.sdkInControl=1
            else:
                self.vehicle.sdkInControl=0
                self.vehicle.vehicleControl.sdkInControl=0
            self.vehicle.rollCommand=((((self.inputStream.inStream.int8_t())*4.7)-500)/6.66667)
            self.vehicle.pitchCommand=((((self.inputStream.inStream.int8_t())*4.7)-500)/6.66667)
            self.vehicle.powerCommand=self.inputStream.inStream.uint16_t()
            self.vehicle.dataTimeStamp=Constants.getTimestamp()
            self.vehicle.speedCmsINS=math.sqrt(math.pow(self.vehicle.speedXCms,2)+math.pow(self.vehicle.speedYCms,2))
            self.vehicle.speed3DIns=math.sqrt(math.pow(self.vehicle.speedCmsINS,2)+math.pow(self.vehicle.varioRateCms,2))
            #print("Kinematic Packet",self.vehicle.attitude.rollDeg,self.vehicle.attitude.pitchDeg,self.vehicle.attitude.yawDeg,
            #                         self.vehicle.attitude.gyroYaw,self.vehicle.attitude.gyroPitch,self.vehicle.attitude.gyroYaw)
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_CAL):
            self.vehicle.calibType=self.inputStream.inStream.uint8_t()
            self.vehicle.memType=self.inputStream.inStream.uint8_t()
          #  print("Calibration Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_PID):
            pntr=self.inputStream.inStream.uint8_t()
            if(pntr<Constants.PID_POINTERS.MAX_NO_OF_PID_PACKETS):
                self.vehicle.pidData[pntr].P=self.inputStream.inStream.float()
                self.vehicle.pidData[pntr].I=self.inputStream.inStream.float()
                self.vehicle.pidData[pntr].D=self.inputStream.inStream.float()
            else:
                self.vehicle.declination=self.inputStream.inStream.float()
          #  print("Pid Input Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_POINT):
            pntr=self.inputStream.inStream.uint8_t()
            lat=self.inputStream.inStream.int32_t()
            lon=self.inputStream.inStream.int32_t()
            alt=self.inputStream.inStream.uint16_t()
            type=self.inputStream.inStream.uint8_t()
            if(pntr<Constants.WL_LOCATION_POINTER.LOC_PNTR_WAYPOINT):
                if(pntr==Constants.WL_LOCATION_POINTER.LOC_PNTR_CURRENT):
                    self.vehicle.tripMeterMeters=alt
                    self.vehicle.currentPosition.altitude=self.vehicle.altitudeAGLcm
                    self.vehicle.currentPosition.latitude=float(lat)/10000000
                    self.vehicle.currentPosition.longitude=float(lon)/10000000
                    self.vehicle.currentPosition.type=Constants.WL_TERMINATION_TASK.CURRENT_WP
                elif(pntr==Constants.WL_LOCATION_POINTER.LOC_PNTR_HOME):
                    self.vehicle.homePosition.latitude=float(lat)/10000000
                    self.vehicle.homePosition.longitude=float(lon)/10000000
                    self.vehicle.homePosition.altitude=alt
                    self.vehicle.homePosition.type=Constants.WL_TERMINATION_TASK.HOME_WP
                elif(pntr==Constants.WL_LOCATION_POINTER.LOC_PNTR_TARGET):
                    self.vehicle.targetPosition.latitude=float(lat)/10000000
                    self.vehicle.targetPosition.longitude=float(lon)/10000000
                    self.vehicle.targetPosition.altitude=alt
                    self.vehicle.targetPosition.type=Constants.WL_TERMINATION_TASK.TARGET_WP
            else:
                self.vehicle.wayPointList[pntr-Constants.WL_LOCATION_POINTER.LOC_PNTR_WAYPOINT].latitude=lat
                self.vehicle.wayPointList[pntr-Constants.WL_LOCATION_POINTER.LOC_PNTR_WAYPOINT].longitude=lon
                self.vehicle.wayPointList[pntr-Constants.WL_LOCATION_POINTER.LOC_PNTR_WAYPOINT].altitude=alt
                self.vehicle.wayPointList[pntr-Constants.WL_LOCATION_POINTER.LOC_PNTR_WAYPOINT].type=type
          #  print("Point Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_GPCG):
            self.vehicle.targetWaypointNumber=self.inputStream.inStream.uint8_t()
            self.vehicle.angleToTgtDeg=float(self.inputStream.inStream.uint16_t())*(1.8/math.pi)
            self.vehicle.gpsCourse=float(self.inputStream.inStream.uint16_t())*(1.8/math.pi)
            self.vehicle.distanceToHomeMeters=self.inputStream.inStream.uint16_t()
            self.vehicle.distanceToTargetMeters=self.inputStream.inStream.uint16_t()
            self.vehicle.gpsAccuracyMeters=self.inputStream.inStream.uint16_t()
            sats=self.inputStream.inStream.uint8_t()
            self.vehicle.maxNoOfSats=(sats & 0x3F)
            if((sats & 0x40)>0):
                self.vehicle.gpsGotHomePoint=True
            else:
                self.vehicle.gpsGotHomePoint=False
            if((sats & 0x80)>0):
                self.vehicle.gpsLocked=True
            else:
                self.vehicle.gpsLocked=False
            self.vehicle.insRelocated=self.inputStream.inStream.uint16_t()
         #   print("GPCG Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_CPP):
            print("CPP Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_COP_PKT):
            print("COP Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_CNTRL_PKT):
            print("CNTRL Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_PIDOP_PKT):
            self.vehicle.internalParams.outputPIDPower=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.gpsCorrXDegs=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.gpsCorrYDegs=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.rollSpdDes=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.pitchSpdDes=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.pitchIntErr=self.inputStream.inStream.int16_t()
            self.vehicle.internalParams.rollIntErr=self.inputStream.inStream.int16_t()
          #  print("PID OUTPUT Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_AUX1_PKT):
            self.vehicle.advancedParams.status=self.inputStream.inStream.uint8_t()
            self.vehicle.advancedParams.baroRate=self.inputStream.inStream.int16_t()
            self.vehicle.advancedParams.lidarHt=self.inputStream.inStream.int16_t()
            self.vehicle.advancedParams.magX=self.inputStream.inStream.int16_t()
            self.vehicle.advancedParams.magY=self.inputStream.inStream.int16_t()
            self.vehicle.advancedParams.magZ=self.inputStream.inStream.int16_t()
          #  print("AUX1 Packet")
        elif(self.protocol.commandType==Constants.WL_PACKETS.PKT_AUX1_CONF):
            print("AUX1 CONF Packet")
      

    def clearTimings(self):
        self.parsedHomePoint=False
        self.vehicle.attitude.pSampleTime=0
        self.vehicle.pDataTime=0
        self.protocol.pRcvTime=0
        self.protocol.dataArrvied=False

    def copyToOutputBuffer(self,data):
        self.outputBuffer=data

    def writeToPacketToBuffer(self,packetId,packetLen):
        tempBuff=[]
        self.protocol.chkA=self.protocol.chkB=0
        tempBuff.append(chr(Constants.WL_PROTOCOL_ATTRIBUTES.HEADER1))
        tempBuff.append(chr(Constants.WL_PROTOCOL_ATTRIBUTES.HEADER2))
        tempBuff.append(chr(packetId))
        tempBuff.append(chr(packetLen))
        self.protocol.incrementCheckSums(packetId)
        self.protocol.incrementCheckSums(packetLen)
        #print(packetLen," chk ",len(self.outputBuffer))
        for i in range(packetLen):
            tempBuff.append(self.outputBuffer[i])
            self.protocol.incrementCheckSums(ord(self.outputBuffer[i]))
        tempBuff.append(chr(self.protocol.chkA))
        tempBuff.append(chr(self.protocol.chkB))
        self.protocol.sendBuffer.append(tempBuff)
        self.protocol.sendLength+=1

    def processPeriodicalDataFetch(self):
        arm=0
        if((Constants.millis()-self.pSDKSendTime)>=Constants.SDK_UPDATE_TIME):
            self.pSDKSendTime=Constants.millis();
            self.tgtSendCnts+=1
            if(self.tgtSendCnts>=3):
                self.tgtSendCnts=0;
                if(self.vehicle.assignTargetPoint>0):
                    self.vehicle.assignTargetPoint-=1
                    self.vehicle.targetPosition.trueTargetAltitude=self.vehicle.targetAlt*100;
                    self.packetGen.generateTargetLocationPacket(self.vehicle.targetLat,self.vehicle.targetLon,self.vehicle.targetAlt);
                    self.copyToOutputBuffer(self.packetGen.outputBuffer)
                    self.writeToPacketToBuffer(Constants.WL_PACKETS.PKT_POINT,Constants.WL_PACKETS_PL.POINT_PL)
                    time.sleep(0.03);

            if(self.vehicle.vehicleControl.takeControl==1):
                self.vehicle.setMode=0;
                if(self.pTakeControl!=self.vehicle.vehicleControl.takeControl):
                    self.vehicle.assignMode=self.vehicle.currentFlightMode;
                if(self.vehicle.vehicleControl.armSendCnts>0):
                    arm=1;
                    self.vehicle.vehicleControl.armSendCnts=self.vehicle.vehicleControl.armSendCnts-1;
               ## print("MODE: ",self.vehicle.assignMode,self.vehicle.currentFlightMode);
                self.packetGen.generateCommandPacket(self.vehicle.assignMode,self.vehicle.vehicleControl.angleX,self.vehicle.vehicleControl.angleY,self.vehicle.vehicleControl.yaw,self.vehicle.vehicleControl.power,self.vehicle.vehicleControl.takeControl,arm);
                self.copyToOutputBuffer(self.packetGen.outputBuffer)
                self.writeToPacketToBuffer(Constants.WL_PACKETS.PKT_CNTRL_PKT,Constants.WL_PACKETS_PL.CNTRL_PL)
            elif(self.vehicle.setMode==1):
                 self.vehicle.setMode=0;
                 self.packetGen.generateCommandPacket(self.vehicle.assignMode,0,0,0,self.vehicle.powerCommand,0,0);
                 self.copyToOutputBuffer(self.packetGen.outputBuffer)
                 self.writeToPacketToBuffer(Constants.WL_PACKETS.PKT_CNTRL_PKT,Constants.WL_PACKETS_PL.CNTRL_PL)
            self.pTakeControl=self.vehicle.vehicleControl.takeControl;

        if((Constants.millis()-self.pHeatBeatFetch)>=Constants.HEATBEAT_PACKET):
            self.pHeatBeatFetch=Constants.millis()
            self.packetGen.generateCalibrationPacket(int(Constants.WL_CALIBRATION.NO_CALIBRATION),Constants.WL_MEMORY_WRITE.MEM_NO_COMMAND)
            self.copyToOutputBuffer(self.packetGen.outputBuffer)
            self.writeToPacketToBuffer(Constants.WL_PACKETS.PKT_CAL,Constants.WL_PACKETS_PL.CAL_PL)
        if(((Constants.millis()-self.pTargetPositionFetchTime)>=Constants.TARGET_POSITION_PACKET)and
           (self.vehicle.connectedToAutopilot==True) and
           (self.vehicle.gpsGotHomePoint==True)):
            self.pTargetPositionFetchTime=Constants.millis()
            self.packetGen.generatePacketFetch(Constants.WL_LOCATION_POINTER.LOC_PNTR_TARGET)
            self.copyToOutputBuffer(self.packetGen.outputBuffer)
            self.writeToPacketToBuffer(Constants.WL_PACKETS.PKT_POINT,Constants.WL_PACKETS_PL.POINT_GET_PL)
            if(self.vehicle.gpsLocked==True):
                if(self.vehicle.gpsGotHomePoint==True):
                    if(self.parsedHomePoint==False):
                        self.vehicle.homePosition.latitude=self.vehicle.currentPosition.latitude
                        self.vehicle.homePosition.longitude=self.vehicle.currentPosition.longitude
                        self.vehicle.homePosition.altitude=self.vehicle.currentPosition.altitude
                        self.vehicle.homePosition.type=Constants.WL_TERMINATION_TASK.HOME_WP
                    self.parsedHomePoint=True

        

    def dataStep(self,c):
        if((c==Constants.WL_PROTOCOL_ATTRIBUTES.HEADER1) and (self.protocol.step==0)):
            self.protocol.payloadCounter=0
            self.protocol.step=0
            self.protocol.chkA=0
            self.protocol.chkB=0
            self.protocol.step+=1
            self.protocol.pRcvTime=Constants.millis();
        elif((c==Constants.WL_PROTOCOL_ATTRIBUTES.HEADER2) and (self.protocol.step==1)):
            self.protocol.step+=1
            self.protocol.numberOfBytesPresent=0;
            self.protocol.payLoadLength=0;
            self.protocol.pRcvTime=Constants.millis();
        else:
            if(self.protocol.step==2):
                self.protocol.commandType=c
                self.protocol.incrementCheckSums(c)
                self.protocol.step+=1
            elif(self.protocol.step==3):
                self.protocol.payloadCounter=0;
                self.protocol.payLoadLength=c;
                self.protocol.incrementCheckSums(c);
                if(self.protocol.payLoadLength > Constants.WL_PACKETS_PL.MAX_PACKET_LENGTH):
                    self.protocol.step=0
                else:
                    self.protocol.step+=1
                    self.protocol.rcvBuffer=[]
                    self.protocol.rcvLength=0;
            elif(self.protocol.step==4):
                self.protocol.rcvBuffer.append(c)
                if(self.protocol.payloadCounter>=(self.protocol.payLoadLength-1)):
                    self.protocol.step+=1
                else:
                    self.protocol.payloadCounter+=1
                self.protocol.incrementCheckSums(c);
            elif(self.protocol.step==5):
                #print(self.protocol.chkA,c)
                if(self.protocol.chkA==c):
                    self.protocol.step+=1
                else:
                    self.protocol.chkA=0
                    self.protocol.chkB=0
                    self.protocol.step=0
            elif(self.protocol.step==6):
               # print(self.protocol.chkB,c)
                if(self.protocol.chkB==c):
                    self.protocol.pRcvTime=Constants.millis()
                   # print("GOT PKT: ",self.protocol.commandType);
                    self.protocol.dataArrvied=True
                    self.parseData()
                    self.protocol.step=0
            else:
                self.protocol.step=0
        return self.protocol.dataArrvied


