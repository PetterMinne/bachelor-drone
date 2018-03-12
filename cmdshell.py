from dronekit import connect, Command, LocationGlobal,VehicleMode,LocationGlobalRelative
from pymavlink import mavutil
import time, sys, argparse, math, threading
import socket
#import cv2


################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
home_position_set = False
Close = False;
CommandList =None
CommandNumber = 0
print ("Connecting")
vehicle = connect(connection_string, wait_ready=True)





##Hjelpe meny
def help():
	print("----Meny----")


###Metoder
def connectToCircleDetectionThread():	
    host = "127.0.0.1"
    port = 5000
     
    mySocket = socket.socket()
    mySocket.bind((host,port))
    #mySocket.settimeout(15)
    mySocket.settimeout(15)
    print("\nStarting listening thread")
    while not Close:
        try:
            print("Listening for imagerec-Client") 
            mySocket.listen(1)
            conn, addr = mySocket.accept()
            mySocket.settimeout(None)
            print ("\nConnection from: " + str(addr))
            #recvBallCordinates()
            print("Listening for Ballinfo")
            while not Close:
                try:
                    data = conn.recv(1024).decode()
                    if not data:
                        break
                    current = vehicle.location.global_relative_frame
                    GoTo(current)
                    print ("\nthread: from connected  user: " + str(data))
                except:
                    pass        
            print("thread:Connection dropped")     
            conn.close()
            #mySocket.settimeout(15)
        except:
            print("\nTimeout listen imagerec-Client- Restarting")
            #pass
            #mySocket.settimeout(15)
        mySocket.settimeout(15)
    print("Thread  t1 finished.")

def status():
    print (" Type: %s" % vehicle._vehicle_type)
    print (" Mode: %s" % vehicle.mode)
    print (" Armed: %s" % vehicle.armed)
    print (" System status: %s" % vehicle.system_status.state)
    print (" GPS: %s" % vehicle.gps_0)
    print (" Alt: %s" % vehicle.location.global_relative_frame.alt)

##Move L i Z-akse
def takeoff(height):
    # Load commands
    cmds.clear()
    # takeoff to 10 meter
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, 0);
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, height)
    cmds.add(cmd)
    print("added mission")
    cmds.upload()
    time.sleep(2)
    vehicle.armed = True
    nextwaypoint = vehicle.commands.next
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=height*0.95:
            print ("Reached target altitude")
            
        break
        time.sleep(1)
     #      	time.sleep(1)
    time.sleep(1)
    
def land():
    cmds.clear()
    home = vehicle.location.global_relative_frame
    wp = get_location_offset_meters(home, 0, 0, home.alt)
    #print("Distance to waypoint (%s): %s" % (wp, distance_to_current_waypoint()))
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    wp = get_location_offset_meters(wp, 0, 0, -wp.alt)
    print("Landing at: ")
    pos()
    cmds.upload()
    time.sleep(2)
	#vehicle.armed = True
def clear():
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               45, 255,
                                               0,
                                               0, 0, 0, 0, 0, 0)
def gridSearch():
    cmds.clear()
    #home = vehicle.location.global_relative_frame

    ##takeoff
    home = vehicle.location.global_relative_frame
    #if ( home.alt<4):
    wp = get_location_offset_meters(home, 0, 0, 5)
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon,wp.alt)
    cmds.add(cmd)
    #else:
    #    wp = home
    i = 0
    x = 80
    y = 5
    while(i< 6):
        wp = get_location_offset_meters(wp, x, 0, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        wp = get_location_offset_meters(wp, 0, -y, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        wp = get_location_offset_meters(wp, -x, 0, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        wp = get_location_offset_meters(wp, 0, -y, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        i=i+1

    wp = get_location_offset_meters(wp, 0,2* y*i, 0)
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 17, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    CommandList = cmds
    cmds.upload()
    time.sleep(2)

    # Arm vehicle
    vehicle.armed = True

    # monitor mission execution
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands) and not Close:
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next+1
            CommandNumber = display_seq
            print ("Moving to waypoint %s" % display_seq)
            nextwaypoint = vehicle.commands.next
            time.sleep(2)
        if Close:
            break
    # wait for the vehicle to land
    #while vehicle.commands.next > 0:
     #   time.sleep(1)
    time.sleep(1)
    print("Grid mission done, killing Thread")
    #vehicle.armed = False

def GoTo(pos):
    cmds.clear()
    current = vehicle.location.global_relative_frame
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, pos.lat, pos.lon, current.alt)
    cmds.add(cmd)
    cmds.upload()

def GoToNewPos():
    MAV_CMD_OVERRIDE_GOTO =252
    MAV_GOTO_DO_HOLD =0
    MAV_GOTO_DO_CONTINUE =1
    MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2
    MAV_GOTO_HOLD_AT_SPECIFIED_POSITION= 3
    current = vehicle.location.local_frame
    #cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, pos.lat, pos.lon, current.alt)
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          MAV_CMD_OVERRIDE_GOTO, 1,0, MAV_GOTO_HOLD_AT_CURRENT_POSITION ,mavutil.mavlink.MAV_FRAME_LOCAL_NED , 0, 0, 0, 0)

def pos():
    print("%s" % vehicle.location.global_frame)
    print("%s" % vehicle.location.global_relative_frame)
    print("%s" % vehicle.location.local_frame)  # NED

def resume():
    cmds = CommandList

    cmds.upload()

def excecute():
    cmds.upload()
    time.sleep(2)
    vehicle.armed = True
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next+1
            print ("Moving to waypoint %s" % display_seq)
            nextwaypoint = vehicle.commands.next
        time.sleep(1)
##PX4mode.
def PX4setMode():
    MAV_CMD_DO_SET_MODE = 176
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          MAV_CMD_DO_SET_MODE, 0,
                                               MAV_MODE_AUTO,
                                               0, 0, 0, 0, 0, 0)
##Omregne gpspos.
def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

#Create a message listener for home position fix
#@vehicle.on_message('HOME_POSITION')
#def listener(self, name, home_position):
 #   global home_position_set
  #  home_position_set = True
#####################################

##TO DO  Metoder for Bevegelse

#####################################

##Kommandolkke.
#vehicle.mode= "GUIDED"
time.sleep(1)

cmds = vehicle.commands
cmds.clear()
cmds.upload()
vehicle.airspeed=3
t1 = threading.Thread(target=connectToCircleDetectionThread)
t = threading.Thread(target=gridSearch)
Start_Pos = vehicle.location.global_relative_frame
print("------kommandolokke------")
inn = '*'
while not inn == 'q':
    inn=raw_input("cmd:")
    if inn == 'h':
        help()
    elif inn =='c':
        if not t1.isAlive():
            print("connect")
        
            t1.start()
        else:
            print("Allready started listening thread.")
        
    elif inn == 'o':
        print("trying new move command.")
        GoToNewPos()
    elif inn =='s':
        status()
    elif inn =='t':
        height=raw_input("height?:")
        takeoff(int(height))
        time.sleep(1)
    elif inn =='l': #gridseach bane.
        t = threading.Thread(target=gridSearch)
        t.start()

        #goto(wp)
    elif inn == "mode":
    	#mode =raw_input("mode?")
    	PX4setMode()
    	print("MODE SET" )
    elif inn == "arm":
    	vehicle.armed = True

    elif inn == "disarm":
    	vehicle.armed = False
    elif inn == "land":
    	land()
    elif inn == "clear":
        print("Cleared commands")
        #clear()
        vehicle._master.waypoint_clear_all_send()
        #cmds.clear()
        #cmds.upload()
    elif inn == "stop":
        vehicle.airspeed = 0
        vehicle._master.waypoint_clear_all_send()
        current = vehicle.location.global_relative_frame
        GoTo(current)
    elif inn == "home":
        print("--returning home.")
        GoTo(Start_Pos)
    elif inn == "pos":
        print("---Current Position----")
        pos()
# Disarm vehicle
vehicle.armed = False
Close = True
time.sleep(1)
if t1.isAlive():

    t1.join()
if t.isAlive():
    t.join()
# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)
