#!/usr/bin/env python
print ('Inverse Extended Kinematic Modeling based Controller')

# Import Libraries

import numpy as np
import matplotlib.pyplot as plt
import modules as md


# Parameters 

n=1;
s_max=20;
ts=0.1;
d=0;
alpha = 0*np.pi/6;

u_aux = 0.5
u_d = u_aux
ds = 0.001

# Design Constants

kx = 0.9;
ky = 0.9;
kw = 0.9;

lx = 0.3;
ly = 0.3;
lw = 0.5;

# Defining the path

# Cartesian Path
s = np.arange(0,2*np.pi,ds)
x_p = 5 + 3.5*np.cos(s+np.pi)
y_p = 0 + 2.5*np.sin(s)


#s = np.arange(1,4*np.pi,ds)
#x_p = s
#y_p = 0.8*np.sin(1.*x_p-2.5)

#x_p = s
#y_p = 0.0*np.ones(x_p.size)+0.5*s

#x_p = s
#y_p = 0.0*np.ones(x_p.size)+1

#s = np.arange(1,4*np.pi,ds)
#x_p = s
#y_p = 2*np.tanh(0.9*(x_p-8))+5

## Curvature Radio

Rc_n = np.power(np.square(np.diff(x_p)/ds)+np.square(np.diff(y_p)/ds),1.5)
Rc_d = abs((np.diff(x_p[0:x_p.size-1])/ds)*((np.diff(np.diff(y_p)/ds)/ds))-(np.diff(y_p[0:y_p.size-1])/ds)*((np.diff(np.diff(x_p)/ds)/ds)))

Rc   = (Rc_n[0:Rc_d.size]/(Rc_d+0.0001))
Rc   = np.append(Rc,Rc[-4:-1])

# Angle over the path (phi_p)

phi_p = np.arctan2(np.diff(y_p),np.diff(x_p))

phi_p = np.append(phi_p, phi_p[-1])

for k in range (phi_p.size-1):

        phi_p[k+1]= md.ang_continuos(phi_p[k],phi_p[k+1])

### Reserved Space of Memory

x_t    = np.array([])
y_t    = np.array([])

x_d    = np.array([])
y_d    = np.array([])

nu_x = np.array([])
nu_y = np.array([])
nu_p = np.array([])

phi_d = np.array([])
phi_t = np.array([])

phip_d = np.array([])
phi2p_d = np.array([])

u = np.array([])
v = np.array([])
w = np.array([])

k_p = np.array([],int)

x_r  = np.array([0])
y_r  = np.array([0])
phi  = np.array([0])

x   = np.array(x_r + d*np.cos(phi+alpha))
y   = np.array(y_r + d*np.sin(phi+alpha))

df    = 100; k=0;

kk=0

phi_path=[]


def update_1(i):
    
    ln_1.set_data(xpr[i], ypr[i])
    ln_2.set_data(xpr[:i], ypr[:i])    

    DDMR=ax_3.add_collection(robot_func.DDMR(xpr[i],ypr[i],phi[i]))
    
    return ln_1, ln_2, DDMR

k_p_aux=0

### Algoritmo de Control ####

while (df>0.10 and k<1500) or (k_p[k-100]>=dist.size):        

        dist   = np.around(np.sqrt(np.square(x[k]-x_p)+ np.square(y[k]-y_p)), decimals=4)  
     
        k_p = np.append(k_p, np.argmin(dist))         

        df = np.sqrt(np.square(x[k]-x_p[-1])+np.square(y[k]-y_p[-1]))

#        if k>0 and abs(k_p[k]-k_p[k-1])>(6*u_d*ts/ds):
#                k_p[k]=k_p[k-1]+1
#                print(k_p[k],'In')  

        x_d = np.append(x_d,x_p[k_p[k]])
        y_d = np.append(y_d,y_p[k_p[k]]) 
        
        dist_0 = np.sqrt(np.square(x[0]-x_d[0])+ np.square(y[0]-y_d[0]))                
        
#        The follows is used for limite the initial robot's velocity when it is far
        
        if 0.5*dist_0 < dist[k_p[k]]:
            
            xp_d = 0
            yp_d = 0
            
        else:
            
#            This is an empiric constant used for smoothing the desired velocity
            
            kv = 1/(2.5*u_d)
            
#            By using the tanh square show a better performance in practice
        
            xp_d = u_d*np.cos(phi_p[k_p[k]])* np.square(np.tanh(kv*Rc[k_p[k]]))
            yp_d = u_d*np.sin(phi_p[k_p[k]])* np.square(np.tanh(kv*Rc[k_p[k]]))  
        
#       Control Errors

        x_t = np.append(x_t, x_d[k] - x[k])
        y_t = np.append(y_t, y_d[k] - y[k])

#       Auxiliary Control Law        

        nu_x = np.append(nu_x, xp_d + lx*np.tanh(kx*x_t[k]/lx))        
        nu_y = np.append(nu_y, yp_d + ly*np.tanh(ky*y_t[k]/ly))
        
#        if k>0:
#            
#            nu_x[k]= np.average(nu_x[k-1:k+1])
#            nu_y[k]= np.average(nu_y[k-1:k+1])

                        
        if d==0:
                
                phi_d = np.append(phi_d,np.arctan2(nu_y[k],nu_x[k]))


                if k>0: 
                                            
                    phi_d[k] = md.ang_continuos(phi_d[k-1],phi_d[k])                            
                        
                    phip_d   = np.append(phip_d,(phi_d[k]-phi_d[k-1])/ts)                        

                    if abs(phip_d[k]-phip_d[k-1])>1:
                                                
                        phip_d[k]=phip_d[k-1]
#                                
                    phip_d[k]= np.average(phip_d[k-1:k+1])
                        
                else:                 
                    
                    
                    phip_d   = np.append(phip_d,0)
                    
                               
                phi_t = np.append(phi_t,phi_d[k] - phi[k])
                
                nu_p  = np.append(nu_p, phip_d[k] + lw*np.tanh(kw*phi_t[k]/lw)) 
                
                nu_p[k]= np.average(nu_p[k-1:k+1])
               

        else:
                nu_p = np.append(nu_p,(lw)*np.tanh(((nu_y[k]*np.cos(phi[k])-nu_x[k]*np.sin(phi[k]))/(d*np.cos(alpha)))/lw))

        u = np.append(u,(nu_x[k]*np.cos(phi[k])+ nu_y[k]*np.sin(phi[k])+ nu_p[k]*d*np.sin(alpha)))
        
        w = np.append(w,nu_p[k])

        # Send Commands to the Robot        

        # Differential Drive Movil Robot        
        
        xp   = u[k]*np.cos(phi[k])
        yp   = u[k]*np.sin(phi[k])
        phip = w[k]

        phi      = np.append(phi, phi[k] + phip*ts)        
#        phi[k+1] = angPI.ver(phi[k+1])
#        phi[k+1] = ang2PI.ver(phi[k],phi[k+1])       

        v   = np.append(v,-xp*np.sin(phi[k])+ yp*np.cos(phi[k]))
        
        x_r   = np.append(x_r, x_r[k]+ xp*ts)
        y_r   = np.append(y_r, y_r[k]+ yp*ts)
        
        x   = np.append(x,x_r[k+1]+ d*np.cos(phi[k+1]+alpha))
        y   = np.append(y,y_r[k+1]+ d*np.sin(phi[k+1]+alpha))
        

        k=k+1;   
        
                

t=np.arange(0,x_t.size*ts,ts)

er = np.sqrt(np.square(x_t)+np.square(y_t))

# Robot Structure

plt.rc('text', usetex=True)
plt.rc('font', family='serif')



fg_01 = plt.figure()
ax_01 = plt.gca()

ax_01.plot(x_p,y_p,label='Desired Path')
ax_01.plot(x,y,label='Robot Path')
ax_01.plot(x_d,y_d)
ax_01.plot(x_d[0],y_d[0],'or')

ax_01.grid(True, alpha=0.2,linestyle='-')
ax_01.axis('equal')

plt.title("Linear Velocity")
plt.xlabel('$x$ ' '$[m]$')
plt.ylabel('$y$ ' '$[m]$')

ax_01.legend(loc=0)
plt.tight_layout()

fg_2 = plt.figure()

plt.subplot(2,2,1)

ax_21 =plt.gca()
ax_21.plot(t,u)
ax_21.plot(t,w)
ax_21.grid(True, alpha=0.2,linestyle='-')

plt.subplot(2,2,2)

ax_22 = plt.gca()

ax_22.plot(t,x_t)
ax_22.plot(t,y_t)

ax_22.grid(True, alpha=0.2,linestyle='-')

plt.subplot(2,2,3)

ax_23 = plt.gca()

plt.plot(phi)
ax_23.grid(True, alpha=0.2,linestyle='-')



##fg_3 = plt.figure(3)
##ax_3=plt.gca()
##
##ax_3.plot(phi,label='$\phi$')
##ax_3.plot(phi_path,label='$\phi_{p}$')
##ax_3.plot(phi_d,label='$\phi_{d}$')
##ax_2.grid(True, alpha=0.2,linestyle='-')
##ax_3.legend(loc=0)
##ax_3.grid(True, alpha=0.2,linestyle='-')
##
##fg_4 = plt.figure(4)
##ax_4 = plt.gca()
##
##ax_4.plot(nu_x,label='$\nu_{x}$')
##ax_4.plot(nu_y,label='$\nu_{y}$')
##ax_4.plot(phip_d,label='$\dot{\phi}_d$')
##
##ax_2.grid(True, alpha=0.2,linestyle='-')
##ax_3.legend(loc=0)


plt.show()
