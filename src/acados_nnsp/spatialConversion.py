import math
import numpy as np
def FindS(csX,csY,x,y,s0,s1,e):
    a=s0
    b=s1
    t=0.618
    i=0
    while abs(b-a)>e:
        i=i+1
        L = b-a
        s2=a+t*L
        s1=b-t*L
        xr1 = csX(s1)
        yr1 = csY(s1)
        xr2 = csX(s2)
        yr2 = csY(s2)
        if ((xr1-x)**2+(yr1-y)**2<(xr2-x)**2+(yr2-y)**2):
            b=s2
        elif ((xr1-x)**2+(yr1-y)**2>(xr2-x)**2+(yr2-y)**2):
            a=s1
        else:
            a=s1
            b=s2
    s = (a+b)/2
    return s

def VecAngle360(v1,v2,n):
    x = np.cross(v1,v2)
    c = np.sign(np.dot(x,n)) * np.linalg.norm(x)
    a = np.arctan2(c,np.dot(v1,v2))
    return a

def RefToS(csX,csY,x,y,th,s):
    xRef = csX(s)
    yRef = csY(s)
    #pos = np.array([[x-xRef], [y-yRef]])
    #print(pos)
    thRef = np.arctan2(csY(s,1),csX(s,1))
    #R = np.array([[np.cos(-thRef), np.sin(-thRef)], [np.sin(-thRef), - np.cos(-thRef)]])
    #pos = R@pos
    #print(pos)
    #xe = pos[0]
    #ye = pos[1]
    #print(xRef)
    print(thRef)
    x = x-xRef
    y = y-yRef
    xe = x*np.cos(-thRef) - y*np.sin(-thRef)
    ye = x*np.sin(-thRef) + y*np.cos(-thRef)
    p = np.array([0,0,1])
    u = np.array([csX(s,1),csY(s,1),0])
    v = np.array([np.cos(th), np.sin(th), 0])
    the = VecAngle360(u,v,p)
    #the = th-thRef
    
    return xe, ye ,the

def ComputeCurvature(csX,csY,s):
    Vdppx = csX(s,1)
    Vddppx = csX(s,2)
    Vdppy = csY(s,1)
    Vddppy = csY(s,2)
    K = (Vdppx*Vddppy-Vdppy*Vddppx)/((Vdppx**2+Vdppy**2)**(3/2))
    return K