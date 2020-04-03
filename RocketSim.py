"""
OOP adaption of my "1-dimension physics engine"
Units are are metric (meter)

http://www.rocketmime.com/rockets/rckt_sim.html
http://www.rocketmime.com/rockets/rckt_sim2D.html
"""
import math
import matplotlib.pyplot as plt

class RocketSim():

    g = 9.81    #gravity const
    dt = 0.01   #time step
    idx = 1    #loop counter
    tt = 3000  #ending loop count (sim would end after __ number of loops)

    def __init__ (self, mP, mR, rocketD, chuteD, dragC, impulse, thrust):
        self.mP = mP            #mass of propellent in kg
        self.mR = mR            #mass of rocket (excluding propellant) in kg
        self.rocketD = rocketD
        self.chuteD = chuteD    #diamter in meters of parachute
        self.dragC = dragC      #Drag coeficient
        self.impulse = impulse  #impulse of the rockets motor
        self.thrust = thrust    #in newtons T = mass flow rate * exhaust velocity // these varibales depend on characteristics of the motor
        self.tb = impulse/thrust              #burn time
        self.mD = self.mP/self.tb                       #propellant burn rate
        self.area = math.pi*((0.5*self.rocketD)**2) #area of rocket noze cone

        #arrays for holding flight data
        self.z = [0]                  #height in meters TODO: test if its actually in meters
        self.v = [0]                  #velocity TODO: determine what the units are
        self.a = [0]                  #accleration TODO: what are the units m/s^2 ?
        self.t = [0]                  #loop count "time step" TODO: line up time and loop count to work together to get accurate time scale
        self.ka = [0]                 #air resistence
        self.ap = [1.217]             #air presure/density
        self.f = [0]                  #Force TODO: force of what on the rocket? could be the net force
        self.ma = [self.mP + self.mR] #mass of rocket 

    def tempTiming(self, chuteT, mP2, impulse2, thrust2):
        self.chuteT = chuteT
        self.mP2 = mP2
        self.impulse2 = impulse2
        self.thrust2 = thrust2
        self.tb2 = self.impulse2/self.thrust2
        self.mD2 = self.mP2/self.tb2


    def plot(self):
        plt.xlabel('time x (sec) - axis')
        plt.ylabel('height y (M) - axis')
        plt.title('trajectory')
        #blue position
        plt.plot(self.t, self.z) #graph on x,y
        #orange velocity
        plt.plot(self.t, self.v)
        #Green drag
        plt.plot(self.t, self.ka) #graph on x,y
         #Red air density
        plt.plot(self.t, self.ap) #graph on x,y
        #purple accleration
        plt.plot(self.t, self.a) #graph on x,y
        # NET Force
        plt.plot(self.t, self.f)
        plt.show()

    def printFlightStats(self):
        print("Final Velocity: " + str(self.v[self.idx])) 
        print("max Q: " + str(max(max(self.ka),min(self.ka))))
        print("burn time1: " + str(self.tb))
        print("burn time2: " + str(self.tb2))
        print("flight time: " + str(self.t[self.idx]))
        print("Number of interations: " + str(len(self.t)))

    def run1D(self, s2Start):

        while self.idx < self.tt:

            p = 1.217*(0.9**(self.z[self.idx-1]/1000)) #air presure/density decreasing with gain in height (z)
            self.ap.append(p)

            v0 = self.v[self.idx-1]         #last velocity
            scaledTime = self.idx/self.dt   #??
            t = self.idx*self.dt            #TODO: what is this thing SCALE? dt scaled time sample
            self.t.append(t)    

            if t < s2Start:
                self.cM = self.mR + self.mP2 if t > self.tb else self.mP + self.mP2 + self.mR - self.mD*t #rockets current mass
                self.ma.append(self.cM)

                self.area = self.area if t < self.tb + self.chuteT else self.chuteD*math.pi

                self.dragF = 0 if v0 == 0 else (v0/abs(v0))*0.5*p*self.dragC*self.area*(v0**2)
                self.ka.append(self.dragF)
                netF = -self.dragF-self.cM*self.g if t > self.tb else self.thrust-self.dragF-self.cM*self.g
                self.f.append(netF)
            else:
                self.cM = self.mR if t > s2Start + self.tb2 else self.mP2 + self.mR - self.mD2*t #rockets current mass
                self.ma.append(self.cM)

                self.area = self.area if t < self.tb + self.chuteT else self.chuteD*math.pi

                self.dragF = 0 if v0 == 0 else (v0/abs(v0))*0.5*p*self.dragC*self.area*(v0**2)
                self.ka.append(self.dragF)
                netF = -self.dragF-self.cM*self.g if t > s2Start + self.tb2 else self.thrust2-self.dragF-self.cM*self.g
                self.f.append(netF)

            self.a.append(netF/self.cM)
            self.v.append(self.v[self.idx-1] + self.a[self.idx]*self.dt)
            self.z.append(self.z[self.idx-1] + self.v[self.idx]*self.dt)
            #stops the sim once the rocket object hits the ground
            if self.z[self.idx] <= 0 and self.idx > 1:
                return self.v[self.idx]
                break

            self.idx += 1

    def train(self):
        self.trainStartTime = [] #time second stage fired
        self.trainEndVelocity = [] #ending velocity of flight
        for i in range(15*100,25*100,1):
            self.trainStartTime.append(i/100.0)
            self.trainEndVelocity.append(self.run1D(i/100.0))
            


    def run2D(self):
        #angle of rocket (flight anglefa = arctan(Vx/Vy) -> angle of rocket or the vector object
        #Vmag = sqrt(Vx^2 + Vy^2)
        #drag Fd = 0.5*Cd*Area*P*V^2 broken down into Fdx = Fd*sin(fa) Fdy = = Fd*cos(fa) // side note using fa from previous sim run otherwise wwould become circular
        #thrust force: Ftx = T*sin(fa)  Fty = T*cos(fa) //fa from previous run
        #net forces: Fx = Ftx - Fdx     Fy = Fty - Fdy - M*g
        #Accleration: Accx = Fx/M  Accy = Fy/M
        #Velocity: Vx = Vx(previous) + Accx*dt   Vy = Vy(previous) + Accy*dt  Vmag = sqrt(Vx^2 + Vy^2)
        #Positions: X = X(previous) + Vx*dt   Y = Y(previous) + Vy*dt
        pass
        

myRocket = RocketSim(0.293, 1.5, 3*0.0254, 0.01, 0.75, 234.0, 103.0)
myRocket.tempTiming(11, 0.197, 127.0, 127.3)
#myRocket.run1D(22)
myRocket.printFlightStats()
myRocket.train()
#myRocket.plot()

