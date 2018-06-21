import matplotlib.path as mpath
import matplotlib as mpl
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from matplotlib.path import Path

import numpy as np

from numpy import cos

colors= ['C0', 'C1', 'C2', 'C3']

def CLMR(x,y,phi,dlt):

    l_1 = 0.4

    patches=[]
           
    line_1 = mpatches.FancyArrowPatch((x,y),(x+l_1,y), arrowstyle='-')
    line_2 = mpatches.FancyArrowPatch((x,y-0.15),(x,y+0.15), arrowstyle='-')
    line_3 = mpatches.FancyArrowPatch((x+l_1,y-0.15),(x+l_1,y+0.15), arrowstyle='-')

    line_4 = mpatches.FancyArrowPatch((x-0.1,y+0.15),(x+0.1,y+0.15), arrowstyle='-')
    line_5 = mpatches.FancyArrowPatch((x-0.1,y-0.15),(x+0.1,y-0.15), arrowstyle='-')

    line_6 = mpatches.FancyArrowPatch((x+l_1-0.1,y+0.15),(x+l_1+0.1,y+0.15), arrowstyle='-')
    line_7 = mpatches.FancyArrowPatch((x+l_1-0.1,y-0.15),(x+l_1+0.1,y-0.15), arrowstyle='-')
    
    rot_1 = mpl.transforms.Affine2D().rotate_around(x,y,phi)+ plt.gca().transData
    rot_2 = mpl.transforms.Affine2D().rotate_around(x+l_1,y+0.15,dlt)
    rot_3 = mpl.transforms.Affine2D().rotate_around(x+l_1,y-0.15,dlt)

    line_6. set_transform(rot_2)
    line_7. set_transform(rot_3)

    patches.append(line_1)
    patches.append(line_2)
    patches.append(line_3)
    patches.append(line_4)
    patches.append(line_5)
    patches.append(line_6)
    patches.append(line_7)    

    CLMR = PatchCollection(patches, match_original=True)

    CLMR.set_transform(rot_1)

    return CLMR


def DDMR(x,y,phi,i):

    patches = []

    # Robot Body
    circle = mpatches.Circle((x, y), 0.25,fc=colors[i])
    patches.append(circle)

    # Tire 1
    rect = mpatches.Rectangle([x-0.07, y+0.24-(0.025)], 0.14, 0.051, fc="black")
    patches.append(rect)

    # Tire 2
    rect = mpatches.Rectangle([x-0.07, y-0.24-(0.0245)], 0.14, 0.05, fc="black")
    patches.append(rect)

    # add a Polygon
    polygon = mpatches.Polygon(np.array([(x+0.1,y+0.05), (x+0.1,y-0.05), (x+0.2,y)]), color='black')
    patches.append(polygon)

    DDMR = PatchCollection(patches, match_original=True)

    rot = mpl.transforms.Affine2D().rotate_around(x,y,phi)+ plt.gca().transData

    DDMR.set_transform(rot)

    return DDMR

def Toby(x,y,phi):
    
    patches = []
    
    body= mpatches.Ellipse((x,y), 0.35, 0.7, fc='#0033cc')
    patches.append(body)
    head  = mpatches.Circle((0.15+x,y),0.15, zorder=10, fc='#0066ff')
    patches.append(head)

    Toby = PatchCollection(patches, match_original=True)

    rot = mpl.transforms.Affine2D().rotate_around(x,y,phi)+ plt.gca().transData

    Toby.set_transform(rot)

    return Toby

## For use this fuction is required, the actual angle and past angle
## according to function

def ang_2PI(phi_past,phi_now):

    ang = np.array([])    

    for i in range (phi_now.size):

        a = phi_now[i]-phi_past[i]

        if a>0 and a>1 and phi_past[i]<0 and phi_now[i]>0:

            ang = np.append(ang, phi_now[i] - 2*np.pi)
            

        else:

            if (a<0 and a<-1 and phi_past[i]>0 and phi_now[i]<0):

                ang = np.append(ang, phi_now[i] + 2*np.pi)                

            else:

                ang = np.append(ang,phi_now[i])                

    return ang

def ang_PI(phi):

    ang = np.array([])

    for i in range (phi.size):

        n   = np.fix(phi[i]/np.pi)

        imp = (-(np.power((-1),n))+1)/2

        if n>0:

            ver=imp+n

        else:

            ver=n-imp

        ang = np.append(ang, phi[i]-(ver*np.pi))

    return ang
        
def ang_continuos (ang_past,ang_now):
    
    a = ang_now-ang_past
    
    if abs(a) > 1.8*np.pi:
        
        n = abs(1.1*a)//abs(np.pi)
        
        if a > 1.8*np.pi:
            
            return ang_now - n*np.pi 
            
        if a < 1.8*np.pi:
            
            return ang_now + n*np.pi
    else:
        
        return ang_now
