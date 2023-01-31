import memcache
import dagger
import time
import math


### Initialising memcache client
memc=memcache.Client(['127.0.0.1:11211'],debug=1)

### Pluto connection
t = dagger.PlutoConnection()
t.connect(("192.168.4.1",23))
print(t)

### Setting up Raw RC from dagger (python wrapper)
cmd = dagger.SetRawRC(t)
tk = dagger.SetCommand(t)

class control():
    def __init__(self):
        
        #throttle parameters
        self.kp_throttle=230
        self.kd_throttle=45  
        self.ki_throttle=5  
        self.tim=0

        #roll pid parameters 
        self.kp_roll=5 #9
        self.kd_roll=40 
        self.ki_roll=0

        #pitch pid parameters
        self.kp_pitch=7 
        self.kd_pitch=40 
        self.ki_pitch=0


        ##error terms
        #integral error
        self.ierror_throttle=0
        self.ierror_roll=0
        self.ierror_pitch=0

        #derivative error
        self.prev_error_throttle=0
        self.prev_error_roll=0
        self.prev_error_pitch=0
        
        self.a =0
        self.throt = 0

        #storing kp,ki,kd into cache memory
        kp_values={'kp_t':self.kp_throttle,'kd_t':self.kd_throttle,'ki_t':self.ki_throttle,'kp_r':self.kp_roll,'kd_r':self.kd_roll,'ki_r':self.ki_roll,'kp_p':self.kp_pitch,'kd_p':self.kd_pitch,'ki_p':self.ki_pitch}
        memc.set_multi(kp_values)   ##updating kp values

    def limit(self,input_value, max_value, min_value):

        #Used to hardlimit the maximum and minimum values sent to the drone
        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value
        

    def positionhold(self,x_current,y_current, z_current, x_init, y_init, z_init, ref_throttle):
        
        # This function is used to hold the drone's position at a desired point

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
            iwindup_value=55
            
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
                throttle=1700
        else:
            throttle = 1720
                

        ## setting roll and pitch

        roll = int(self.limit(1510 - self.kp_roll*error_roll-self.kd_roll*derror_roll,1530,1480))

        if abs(error_roll)<1:
            roll=1500

        pitch = int(self.limit(1495 - self.kp_pitch*error_pitch-self.kd_pitch*derror_pitch,1520,1470))
        
        if abs(error_pitch)<1:
            pitch=1515   

        ## updating all values in memcache
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
    c=control()

    # getting the battery level from dagger API
    battery_level=c.batterystatus(t)
    
    ## throttle condition depending on battery level
    if(battery_level<=3.9):
        ref_throttle=1600
    else:
        ref_throttle=1550
        

    ##initial arming
    time.sleep(2)
    cmd.disarm_drone()
    time.sleep(0.2)
    cmd.arm_drone()
    time.sleep(0.1)

    # main control loop
    while True:
        try:
            current_coordinates=memc.get_multi(['x','y','z'])
            c.positionhold(current_coordinates['x'],current_coordinates['y'],current_coordinates['z'],initial_coordinates['x_initial'],initial_coordinates['y_initial'],initial_coordinates['z_initial'], ref_throttle)
        except KeyboardInterrupt:
            tk.command(dagger.CmdType.LAND)
            break

