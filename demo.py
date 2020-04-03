"""
Program that simulates a rocket launch Numerical Implimentation
8-27-19 John Trager 

NOTES: 
for Mass of Popellant (if unknown) use the impulse/800 for black powder motors, and Impulse/1800 to get the weight in Kg
Thrust is the total amount of force the motor will/can put out
netF = thrust - drag - weight
+y V= -g - drag + Thrust --> -g - 0.5PV|V| * CdA/m + V/|V| * (mass flow * exhaust V)/m

"""
import math
import matplotlib.pyplot as plt 


#returns the sign(+ or -) of the input value
sign = lambda x: math.copysign(1, x)

#return all calls end() later
printAll = True

def train():
    """
    train() function loops over trajectory() to tweak where the second motor fires to land rocket
    currently using PID system to adjust different fire times based on simresults
    """
    
    #list holds position of rocket for velocity graph
    V = [] 
    #list of time step values for plot
    ida = []
    #time delay
    xa = []
    #
    error = []
    #adding zeros to all lists
    error.append(0)
    xa.append(0)
    ida.append(0)
    V.append(0)
    #time varible
    idx = 1
    #max time (in seconds)
    max_run = 500
    #time step
    dt = 0.00075
    #PID control vars
    I_error = 0
    kP = 0.01
    kI = 0.0005
    kD = 0.01
    #position
    x = trajectory(100)[1] * 0.75
    #PID train loop for landing rocket (second burn)
    while idx < max_run:
        #position appended to list
        V.append(trajectory(x)[0])
        #error appended to list 
        error.append(0 - V[idx])
        #kI error
        I_error += error[idx] * dt 
        derivative = (error[idx] - error[idx-1]) / dt
        #calculating PIDs
        P = kP * error[idx]
        I = kI * I_error 
        D = kD * derivative
        #total PID system
        Ts = P + I + D
        #Total calcualted through sigmoid function to limit large change
        Tc = 1 / (1 + math.exp(-Ts)) 
        #calculating differet fire time (delay) based on if rocket is fired too early/late
        if V[idx-1] > V[idx]:
            x -= Tc
            xa.append(x)
        else:
            x += Tc
            xa.append(x)
        #increment time
        idx += 1
        # add time to list
        ida.append(idx)
        #print position of each test
        print("X: " + str(x))
    
    plt.plot(ida,V)
    plt.plot(ida, xa)
    #plt.plot(ida, error)
    plt.show()



def end(idx,v,y,tb,tb2,t,ka,ta,Pa,a,Ma,Fa):
    """
    end() takes in many different list which stored flight data
    and uses matplotlib to graph the data
    """

    print("Final Velocity: " + str(v[idx-1]))    
    print("max Q: " + str(max(max(ka),min(ka))))
    print("burn time1: " + str(tb))
    print("burn time2: " + str(tb2))
    print("flight time: " + str(t))
    print("printing graph")

    plt.xlabel('time x - axis')
    plt.ylabel('height y - axis')
    plt.title('trajectory')
    #blue position
    plt.plot(ta, y) #graph on x,y
    #orange velocity
    plt.plot(ta, v) #graph on x,y
    #Green drag
    plt.plot(ta, ka) #graph on x,y
    #Red air density
    plt.plot(ta, Pa) #graph on x,y
    #purple
    plt.plot(ta, a) #graph on x,y
    #   Force
    plt.plot(ta, Fa)
    
    plt.show()

    #blue
    plt.plot(ta, y)
    #orange
    plt.plot(ta, v)
    #Green
    plt.plot(ta, a) #graph on x,y
    #red
    plt.plot(ta, Fa)

    plt.show()
    #ta
def trajectory(xd):
    """
    trajectory() takes in time delay for the second motor
    Claculates iterative the trajectory of the rocket over time
    Really where are the physics is going on
    """
    
    #Settings for launch
    dt = 0.01 #time step
    tc = 11 #chute delay time
    tt = 20000 #total time/loop count
    Mp = 0.293 #kg mass of propellant
    Mr = 1.5 #intial mass of rocket in kg
    g = 9.81 #gravity const
    Dr = 0.0254*3 #(in meters takes in inches)diameter of rocket for area of nose cone for calc drag
    ChD = 0.01 #0.0254*28 <- inches to meters diameter of chute | curently 0.1 meter chute diameter
    Cd = 0.75 #Drag coeficient
    A = math.pi*((0.5*Dr)**2) #area of rocket noze cone
    ChA = math.pi*ChD*ChD/4 #chute area
    Im = 234 #impulse of motor
    Th = 103  #thrust in newtons T = mass flow rate * exhaust velocity // these varibales depend on characteristics of the motor
    tb = Im/Th 
    Md = Mp/tb

    #second stage de-cell
    Mp2 = 0.197
    Im2 = 127.0
    Th2 = 127.3
    tb2 = Im2/Th2
    Md2 = Mp2/tb
    t2_start = True
    t2_delay = xd

    idx = 1 #loop counter
    id2 = 1
    #lists holding all data for graphing end()
    y = []
    v = []
    a = []
    ta = []
    ka = []
    Pa = []
    Fa = []
    Ma = []
    Ma.append(Mp+Mp2+Mr)
    Fa.append(0)
    Pa.append(1.217)
    ka.append(0)
    ta.append(0)
    y.append(0)
    v.append(0)
    a.append(0)
    
    #Start the simulation Loop

    while idx < tt:

        P = 1.217*(0.9**(y[idx-1]/1000))
        Pa.append(P)
        v0 = v[idx-1]
        t = idx*dt
        ta.append(t) # t but in a list for graphing

        stage2 = True if (t > t2_delay + tb) and (y[idx-2] > y[idx-1]) else False
        
        if not stage2:
        
            #current mass calc derementing for the burn time of the motor
            Mc = Mr + Mp2 if t > tb else Mp + Mp2 + Mr - Md*t
            Ma.append(Mc)
            area = A if t < tb + tc else max(ChA, A)
            #calc drag force mag and direction usind sign(v0)
            k = 0 if v0 == 0 else sign(v0)*0.5*P*Cd*area*(v0**2)
            ka.append(k)
            #calc net F
            F = -k-Mc*g if t > tb else Th-k-Mc*g 
            Fa.append(F)

        else:
            #motor 2 start
            if t2_start:
                id2 = 1
                t2_start = False
            t2 = id2*dt
            #current mass calc derementing for the burn time of the motor
            Mc = Mr if t > t2_delay + tb + tb2  else Mp2 + Mr - Md2*t2
            Ma.append(Mc)
            area = A if t < tb + tc else max(ChA, A)
            #calc drag force mag and direction usind sign(v0)
            k = 0 if v0 == 0 else sign(v0)*0.5*P*Cd*area*(v0**2)
            ka.append(k)
            #calc net F but little confused with the THRUST
            F = -k-Mc*g if t > t2_delay + tb + tb2 else Th2-k-Mc*g
            Fa.append(F)
            id2 += 1 

        #update accleration
        a.append(F/Mc)
        #update velocity
        v.append(v[idx-1] + a[idx]*dt) 
        #update position
        y.append(y[idx-1] + v[idx]*dt)
        #stops once touches ground
        if y[idx] < 0:
            y[idx] = 0
            v[idx] = 0
            break
        idx += 1
    
    #call method that prints results with matlib
    end(idx,v,y,tb,tb2,t,ka,ta,Pa,a,Ma,Fa) if printAll else None
    print("Final Velocity: " + str(v[idx-1]))
    return [v[idx-1], t]


if __name__ == "__main__":
    trajectory(17.185)

