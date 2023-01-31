import memcache
import dagger
import time
import math


##initialising memcache
memc=memcache.Client(['127.0.0.1:11211'],debug=1)

########### Pluto connection
t = dagger.PlutoConnection()
t.connect(("192.168.4.1",23))
print(t)

# setting up Raw RC
cmd = dagger.SetRawRC(t)
tk = dagger.SetCommand(t)


class control():
    def __init__(self):
        
        ##throttle parameters
        self.kp_throttle=220 
        self.kd_throttle=45  
        self.ki_throttle=5  
        self.tim=0

        ## roll pid parameters 
        self.kp_roll=5 
        self.kd_roll=40 
        self.ki_roll=0

        ##pitch pid parameters
        self.kp_pitch=7 
        self.kd_pitch=40 
        self.ki_pitch=0

        ##error terms
        ##integral error
        self.ierror_throttle=0
        self.ierror_roll=0
        self.ierror_pitch=0

        ##derivative error
        self.prev_error_throttle=0
        self.prev_error_roll=0
        self.prev_error_pitch=0

        ##flag for takeoff
        self.flag=0
        
        self.a =0
        self.throt = 0

        ## storing kp,ki,kd into cache memory
        kp_values={'kp_t':self.kp_throttle,'kd_t':self.kd_throttle,'ki_t':self.ki_throttle,'kp_r':self.kp_roll,'kd_r':self.kd_roll,'ki_r':self.ki_roll,'kp_p':self.kp_pitch,'kd_p':self.kd_pitch,'ki_p':self.ki_pitch}
        memc.set_multi(kp_values) 

    def limit(self,input_value, max_value, min_value):

        #Used to hardlimit the maximum and minimum values sent to the drone

        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

    
    def positionhold(self,x_current,y_current, z_current, x_init, y_init, z_init, ref_throttle, vert):

        ##throttle error
        error_throttle = z_init-z_current           ##error
        self.ierror_throttle += error_throttle      ##integral error
        derror_throttle=error_throttle-self.prev_error_throttle     ##derivative error
        self.prev_error_throttle=error_throttle     ##previous error

        ##pitch error
        error_pitch=y_current-y_init                ##error
        self.ierror_pitch+=error_pitch              ##integral error
        derror_pitch=error_pitch-self.prev_error_pitch      ##derivative error
        self.prev_error_pitch=error_pitch           ##previous error

        ##roll error
        error_roll=x_current-x_init                 ##error
        self.ierror_roll+=error_roll                ##integral error
        derror_roll=error_roll-self.prev_error_roll     ##derivative error
        self.prev_error_roll=error_roll             ##previous error
        

        ## changing throttle values for low battery
        if(ref_throttle>=1600):    
            iwindup_value=75
        else:
            iwindup_value=65
            

        # condition for when the drone reaches the destination point
        if(abs(z_current-z_init)<= 0.08 or self.a==1):
            print("reached")
            self.throt = int(self.limit(ref_throttle - self.kp_throttle*error_throttle
                                      - self.limit(self.ki_throttle*self.ierror_throttle,iwindup_value,
                                                   -iwindup_value)-self.kd_throttle*derror_throttle,1850,1350))
            
            throttle = int(self.throt - 0*math.sin(self.tim*25))
            self.tim+=1
            self.a=1
            if(z_current<=1.77):
                throttle = 1500
            elif(z_current>=2.1):
                throttle=1720
        else:
            throttle = 1750
            

        ## setting roll and pitch
        roll = int(self.limit(1500 - self.kp_roll*error_roll-self.kd_roll*derror_roll,1530,1470))

        if abs(error_roll)<1:
            roll=1500

        pitch = int(self.limit(1495 - self.kp_pitch*error_pitch-self.kd_pitch*derror_pitch,1520,1470))

        if(vert==1):
            roll = int(self.limit(1500 - self.kp_roll*error_roll-self.kd_roll*derror_roll,1523,1473))
            pitch = int(self.limit(1495 - self.kp_pitch*error_pitch-self.kd_pitch*derror_pitch,1515,1475))


        if abs(error_pitch)<1:
            pitch=1515        
    
    
        ##updating all values in cache
        plot_values={'error_t':error_throttle,'error_r':error_roll,'error_p':error_pitch,'throttle':throttle,'roll':roll,'pitch':pitch}
        memc.set_multi(plot_values)  

        ## sending control values (roll, pitch and throttle using dagger API)
        cmd.set_rct(roll, pitch, throttle)
        time.sleep(0.15)    

    def batterystatus(self,t):
        # used to get battery status from python wrapper
        analog = dagger.Analog(t)
        data = analog.get_analog_data()
        return data.vbat/10
        

if __name__ == '__main__' :

    # getting initial coordinates from memcache
    initial_coordinates=memc.get_multi(['x_initial','y_initial','z_initial'])

    # setting rectangle corners as waypoints
    p1=[initial_coordinates['x_initial'],initial_coordinates['y_initial'],initial_coordinates['z_initial']]
    p2=[p1[0]-200,p1[1],p1[2]]
    p3=[p1[0]-200,p1[1]+100,p1[2]]
    p4=[p1[0],p1[1]+100,p1[2]]
    c=control()

    # getting the battery level from dagger API
    battery_level=c.batterystatus(t)

    ## throttle condition depending on battery level
    if(battery_level<=3.9):
        ref_throttle=1600 ## 1600
    else:
        ref_throttle=1550 


    ##initial arming
    time.sleep(2)
    cmd.disarm_drone()
    time.sleep(0.2)
    cmd.arm_drone()
    time.sleep(0.1)

    start_time=time.time()
    a=10
    vert=0
    flag=1
    count =0
    count_num = 4

    # main control loop
    while True:
        try:
            # getting realtime coordinates from tracker using memcache
            current_coordinates=memc.get_multi(['x','y','z'])

            # takeoff from starting point (p1)
            if abs(initial_coordinates['z_initial']-current_coordinates['z'])>=0.05 and flag==1:
                ##initial takeoff
                if(count<7):
                    c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],initial_coordinates['x_initial'],initial_coordinates['y_initial'],initial_coordinates['z_initial'],ref_throttle,vert)
                    count+=1
                if count==7:
                    flag=0
                    count=0
                    a=0

            # going to first waypoint (p2)
            elif a==0:
                vert=0
                c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p2[0],p2[1],p2[2],ref_throttle,vert)

                # condition for when drone is hovering at starting point
                if abs(current_coordinates['x']-p2[0]) < 15:
                    if(count<count_num):
                        c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p2[0],p2[1],p2[2],ref_throttle,vert)
                        count+=1
                    if count==count_num:
                        a=1
                        flag=0
                        count=0

            # going to second waypoint (p3)
            elif a==1:
                vert=1 
                c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p3[0],p3[1],p3[2],ref_throttle, vert)
                if abs(current_coordinates['y']-p3[1]) < 15:
                    if(count<count_num):
                        c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p3[0],p3[1],p3[2],ref_throttle, vert)
                        count+=1
                    if count==count_num:
                        a=2
                        flag=0
                        count=0

            # going to third waypoint (p4)
            elif a==2:
                vert=0
                c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p4[0],p4[1],p4[2],ref_throttle, vert)
                if abs(current_coordinates['x']-p4[0]) < 25:
                    if(count<3):
                        c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p4[0],p4[1],p4[2],ref_throttle, vert) 
                        count+=1 
                    if count==3:
                        a=3
                        flag=0
                        count=0

            # going back to starting point (p1) and landing 
            elif a==3:
                vert=1 
                c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],p1[0],p1[1],p1[2],ref_throttle, vert)
                flag=0
                # landing
                if abs(current_coordinates['y']-p1[1]) < 15:
                    tk.command(dagger.CmdType.LAND)


        # failsafe button     
        except KeyboardInterrupt:
            tk.command(dagger.CmdType.LAND)
            break
