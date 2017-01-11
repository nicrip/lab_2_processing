from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=3, suppress=True)

from itertools import product, combinations

from math import sin, cos, sqrt, pi

#draw a vector
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from mixermagics import *

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

        
def drawThr(ax, q, p, f, clr="b"):
    x,y,z = SE3_v((q,p),(f,0,0))
    dra = Arrow3D([p[0],x],[p[1],y],[p[2],z], mutation_scale=20, lw=2, arrowstyle="-|>", color=clr)
    ax.add_artist(dra)
    
def drawThusters(conf, ax, fth):
    drawThr(ax, conf.q12, conf.p1, fth[0])
    drawThr(ax, conf.q12, conf.p2, fth[1])
    drawThr(ax, conf.q34, conf.p3, fth[2])    
    drawThr(ax, conf.q34, conf.p4, fth[3])
    drawThr(ax, conf.q56, conf.p5, fth[4])
    drawThr(ax, conf.q56, conf.p6, fth[5])
