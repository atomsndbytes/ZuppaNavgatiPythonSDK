import time
import Constants
import ZuppaNavgatiSDK as sdk

#print(int((Constants.WL_CALIBRATION.NO_CALIBRATION)),type(Constants.WL_CALIBRATION.NO_CALIBRATION))

#AP(SECONDARY SLAVE) <-> COMPANION COMPUTER(SLAVE) < - > GCS(MaSTER) 
#port=sdk.PrimaryInterface("COM24","COM25") # <Autopilot com port> , <Telemetry module com Port>

#AP(SLAVE) <-> COMPANION COMPUTER(MASTER)
port=sdk.PrimaryInterface("COM24",None) # <Autopilot com port> , <Telemetry module com Port>

#-------------------APPLICATION CODE-----------------
def pwmToDeg(inputPwm):
    rel=((inputPwm-1500)/500)*70;
    return rel
tog=0
while(True):
    
    #print("Check: ",port.forwardingTimeout);
    #if(port.forwardingTimeout==True): # CONVERTS Schema from 
    #    port.usingForwarding=False;   # AP(SECONDARY SLAVE) <-> COMPANION COMPUTER(SLAVE) <-> GCS(MaSTER)  TO   AP(SLAVE) <-> COMPANION COMPUTER(MASTER)
    
    #port.fwdConn.write("DaTA FROM VISION SYSTEM");

    # Common Access port.wlInterface.vehicle
    # Roll - 15 , pitch - 10 , yaw - 20 dps 
   
    #FOR DIRECT ROLL < PITCH YAW
   # port.wlInterface.vehicle.setFlightMode(Constants.FLIGHT_MODES.EZ_FLY);                 # FIXED WING -- heading hold
  #  port.wlInterface.vehicle.vehicleControl.assignControl(pwmToDeg(1700),pwmToDeg(1700),20,1500); # FIXED WING  #20HZ can be given
    
    
    
    # FOR TARGET POINT METHOD
   # port.wlInterface.vehicle.setFlightMode(Constants.FLIGHT_MODES.NAVIGATION);    # FIXED WING
   # port.wlInterface.vehicle.setFlightMode(Constants.FLIGHT_MODES.POSITION_HOLD); # MULTI COPTER
   # time.sleep(0.1);
   # port.wlInterface.vehicle.setTargetPoint(123456788,801234567,100); # 15 times a second ie: 15hz

    #port.wlInterface.vehicle
    if(tog==0):
        tog=1;
        port.wlInterface.vehicle.vehicleControl.assignControl(10,20,30,1500);
        port.wlInterface.vehicle.setFlightMode(Constants.FLIGHT_MODES.POSITION_HOLD);
    else:
        tog=0;
        port.wlInterface.vehicle.setFlightMode(Constants.FLIGHT_MODES.MANUAL);
        #port.wlInterface.vehicle.vehicleControl.withDrawControl();
        port.wlInterface.vehicle.vehicleControl.toggleArm();

        port.wlInterface.vehicle.setTargetPoint(123456789,801234567,100); #The target position should not have any decimal places and maximum length on 10 digits
                                                                          #12.3456789 , 80.1234567 is 123456789,801234567

    print(port.wlInterface.vehicle.attitude.rollDeg,port.wlInterface.vehicle.attitude.pitchDeg,port.wlInterface.vehicle.attitude.yawDeg)
    print(port.wlInterface.vehicle.attitude.gyroPitch,port.wlInterface.vehicle.attitude.gyroRoll,port.wlInterface.vehicle.attitude.gyroYaw)
    print(port.wlInterface.vehicle.connectedToAutopilot,port.wlInterface.vehicle.currentFlightMode,port.wlInterface.vehicle.armedStatus,port.wlInterface.vehicle.dataTimeStamp)
    time.sleep(3)