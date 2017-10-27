import matplotlib.path as mpath
import matplotlib as mpl
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

import numpy as np

from numpy import cos

def DDMR(x,y,phi):

    patches = []

    # Robot Body
    circle = mpatches.Circle((x, y), 0.25,fc="#ff751a")
    patches.append(circle)

    # Tire 1
    rect = mpatches.Rectangle([x-0.07, y+0.24-(0.025)], 0.14, 0.051, fc="black")
    patches.append(rect)

    # Tire 2
    rect = mpatches.Rectangle([x-0.07, y-0.24-(0.0245)], 0.14, 0.05, fc="black")
    patches.append(rect)

    # add a Polygon
    polygon = mpatches.Polygon(np.array([(x+0.1,y+0.05), (x+0.1,y-0.05), (x+0.2,y)]), color="#00ff00")
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

def ang_2PI(phi_past,phi_actual):

    a = phi_actual-phi_past

    if a>0 and a>1 and phi_past<0 and phi_actual>0:

        ang = phi_actual - 2*np.pi

        return ang

    else:

        if (a<0 and a<-1 and phi_past>0 and phi_actual<0):

            ang = phi_actual + 2*np.pi

            return ang

        else:

            ang = phi_actual

            return ang

def ang_PI(phi):

    n   = np.fix(phi/np.pi)

    imp = (-(np.power((-1),n))+1)/2

    if n>0:

        ver=imp+n

    else:

        ver=n-imp

    ang = phi-(ver*np.pi)

    return ang
