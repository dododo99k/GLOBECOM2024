import numpy as np
import os, string, sys, time

#####################################################################

RANDOMS = 0.0 # the min (1-RANDOMS) and max (1+RANDOMS) regarding vehicle and server capacity, kind of system dynamics

MINI_ALLOC_RADIO = 0.05
MINI_ALLOC_COMP = 0.1


def save_subfig(fig,ax,fig_name,save_path = './'):
    scale = 2
    bbox = ax.get_tightbbox(fig.canvas.get_renderer()).expanded(scale, scale)
    extent = bbox.transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(save_path+fig_name, bbox_inches=extent)

def fix_hist_step_vertical_line_at_end(ax):
    import matplotlib
    axpolygons = [poly for poly in ax.get_children() if isinstance(poly, matplotlib.patches.Polygon)]
    for poly in axpolygons:
        poly.set_xy(poly.get_xy()[:-1])