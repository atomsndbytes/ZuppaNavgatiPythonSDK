#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#---------Python 3.8.8
#Author : Venkatesh Sai
#Ts     : 12:05p.m / 04-04-2022
#© Copyrighted (2021) : Venkatesh Sai , Zuppa Geo Navigation Technologies Pvt. Ltd . Github Repo : f434a20243df049cc079d2ab2e5cf9b24a0170df
import time
from importlib import reload
import serial
import threading
import serial.tools.list_ports
import struct
import enum
import binascii
import Constants
import numpy as np
import MessageInterface


def reloadFLIRSerialLib():
    Constants.SERIAL_LIB_FLIR=reload(serial)

class PrimaryInterface:

    def listComs(self):
        ports = list(serial.tools.list_ports.comports())
        return ports

    def checkIfPortsExists(self, port):
        ret=False
        self.comList=self.listComs();
        if(self.comList != None):
            if(len(self.comList)>0):
                for p in self.comList:
                    #print("FIND: "+str(p.device))
                    if((str(p.device)==port)):
                        #print("FOUND")
                        self.comPresent=True
                        ret=True
        return ret

 
    def closePort(self):
        try:
            self.conn.close()
            if(self.fwdConn!=None):
                self.fwdConn.close()
            serial=reload(serial)
        except Exception as error:
            a=1

    def __init__(self,serialNo,serialFwd):
        self.inData=[]
        self.fwdData=[]
        self.arrData=[]
        self.sno=serialNo
        self.wlInterface=MessageInterface.WLMessageParser()
        self.data = None
        self.comPresent=False;
        self.killTime=0;
        self.validData=False
        self.stopped = False
        self.currentState=0
        self.pState=0
        self.conn = None
        self.fwdConn=None
        self.fstConn=False
        self.pFwdRcvTime=0
        self.forwardingTimeout=True
        self.fwdCom=None
        if(serialFwd):
            self.usingForwarding=True
            self.forwardingTimeout=False
            self.fwdCom=serialFwd
        else:
            self.usingForwarding=False
        self.com=serialNo
        self.checkIfPortsExists(self.com)
        if(self.comPresent==True):
            if(self.fwdCom!=None):
                self.comPresent=False
                self.checkIfPortsExists(self.fwdCom)
                if(self.comPresent==False):
                    print("ERROR FORWARD PORT: ",self.fwdCom,"Is not Found!!")
                    self.forwardingTimeout=True
                    self.usingForwarding=False
            if((self.fwdCom == None) or (self.comPresent==True)):
                print("STARTING ZUPPA UART SERVICE ON: "+str(self.com))
                self.closePort()
                self.conn = Constants.SERIAL_LIB_FLIR.Serial(self.com, Constants.DEFAULT_BAUDRATE)
                self.conn.setDTR(0) ## To notify TAU2 that Data Ready To transmit
                self.conn.setRTS(0) ## to Notify TAU2 that Receive Ready
                self.conn.flushInput()
                self.conn.flushOutput()
                self.conn.timeout=0
                if(self.fwdCom!=None):
                    self.pFwdRcvTime=Constants.millis()
                    print("STARTING ZUPPA FORWARD UART SERVICE ON: "+str(self.fwdCom))
                    self.fwdConn = Constants.SERIAL_LIB_FLIR.Serial(self.fwdCom, Constants.DEFAULT_BAUDRATE)
                    self.fwdConn.setDTR(0) ## To notify TAU2 that Data Ready To transmit
                    self.fwdConn.setRTS(0) ## to Notify TAU2 that Receive Ready
                    self.fwdConn.flushInput()
                    self.fwdConn.flushOutput()
                    self.fwdConn.timeout=0
                self.validCom=True
                self.writeMsg=False
                self.writeMsgBytes=None
                self.dataReceived=False
                self.fstConn=True
            self.start()
        else:
            self.validCom=False
            self.writeMsg=False
            self.writeMsgBytes=None
            self.dataReceived=False
            print("FAILED TO START UART SERVICE ON "+str(self.com))



    def writeToUart(self,bytes):
        ret=False
        if(self.validCom==True):
            if(self.conn):
                if(self.writeMsg==False):
                    self.writeMsgBytes=bytes
                    self.writeMsg=True
                    ret=True
        return ret


    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self
    #DATA: $,40,4,-78,0*
    def monitor(self):
        while True:
            try:
                a=1
            except Exception as error:
                print("ERROR")
            time.sleep(0.1)


    def update(self):
        while True:
            if(True):
                if self.stopped:
                    return
                if (self.validCom == True):
                    self.killTime=0;
                    inData=self.conn.read_all()
                    if((len(inData)>0) and (inData!=None)):
                      #  print(inData,len(inData))
                        if(self.usingForwarding==True):
                            if(self.fwdConn!=None):
                                self.fwdConn.write(inData)
                                self.fwdConn.flush()
                              

                        for c in inData:
                            if(self.wlInterface.dataStep(int(c))):
                                if(self.validData==False):
                                    self.wlInterface.clearTimings()
                                self.validData=True
                                self.wlInterface.vehicle.connectedToAutopilot=True
                    if(self.usingForwarding==True):
                        self.fwdData=self.fwdConn.read_all()
                        if((len(self.fwdData)>0) and (self.fwdData!=None)):
                            self.pFwdRcvTime=Constants.millis()
                            self.conn.write(self.fwdData)
                        if((Constants.millis()-self.pFwdRcvTime)>Constants.FWD_PORT_TIMEOUT):
                            self.forwardingTimeout=True
                   
                    self.wlInterface.processPeriodicalDataFetch()

                    if(self.validData):
                        diff=(Constants.millis()-self.wlInterface.protocol.pRcvTime)
                       # print("time",diff)
                        if((Constants.millis()-self.wlInterface.protocol.pRcvTime)>Constants.TELEM_LOST_TIMEOUT):
                            self.validData=False
                            self.wlInterface.vehicle.connectedToAutopilot=False
                            self.wlInterface.protocol.step=0
                            self.wlInterface.protocol.payLoadLength=0
                       
                    if(self.wlInterface.protocol.sendLength>0):
                        self.arrData=[]
                        for sendStr in self.wlInterface.protocol.sendBuffer:
                            for c in sendStr:
                                val=ord(c)
                                self.arrData.append(val)
                            if(len(self.arrData)>0):
                                self.conn.write(self.arrData)
                                self.conn.flush()
                            time.sleep(0.1)
                        self.wlInterface.protocol.sendBuffer=[]
                        self.wlInterface.protocol.sendLength=0
                else:
                    self.comPresent=False;
                    self.checkIfPortsExists(self.com)
                    print("ZUPPA UART SERVICE ON: "+str(self.com))
                    if(self.comPresent==True):
                        self.closePort()
                        reloadFLIRSerialLib()
                        time.sleep(3)
                        self.conn = Constants.SERIAL_LIB_FLIR.Serial(self.com, Constants.DEFAULT_BAUDRATE)
                        self.validCom=True
                        self.conn.setDTR(0) ## To notify TAU2 that Data Ready To transmit
                        self.conn.setRTS(0) ## to Notify TAU2 that Receive Ready
                        self.conn.flushInput()
                        self.conn.flushOutput()
                        self.conn.timeout=0
                        self.validCom=True
                        self.writeMsg=False
                        self.writeMsgBytes=None
                        self.dataReceived=False
                        self.fstConn=True
                        if(self.fwdCom!=None):
                            self.comPresent=False
                            self.checkIfPortsExists(self.fwdCom)
                            if(self.comPresent==False):
                                print("ERROR FORWARD PORT: ",self.fwdCom,"Is not Found!!")
                                self.forwardingTimeout=True
                            else:
                                print("STARTING ZUPPA FORWARD UART SERVICE ON: "+str(self.fwdCom))
                                self.fwdConn = Constants.SERIAL_LIB_FLIR.Serial(self.fwdCom, Constants.DEFAULT_BAUDRATE)
                                self.fwdConn.setDTR(0) ## To notify TAU2 that Data Ready To transmit
                                self.fwdConn.setRTS(0) ## to Notify TAU2 that Receive Ready
                                self.fwdConn.flushInput()
                                self.fwdConn.flushOutput()
                                self.fwdConn.timeout=0

            else:
                print(error)
                self.validCom=False
                self.validData=False
                self.closePort()
            
            time.sleep(0.01)

    def list_com(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
               print(str(p),p.name)
        time.sleep(1)

    def stop(self):
        self.stopped = True


