#!/usr/bin/env python3
"""Measure static-wall consistency across real SLAM map snapshots.

Planes are fitted once in the earliest map. Later maps use the SAME world ROI
and plane, not per-map ICP registration. This is a local consistency diagnostic,
not a ground-truth/global accuracy or loop-closure test.
"""
import argparse
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument("snapshots",nargs="+")
parser.add_argument("--output",required=True)
args=parser.parse_args()
data=[np.load(path) for path in args.snapshots]
reference=data[0]["points"].astype(float)
# The start area contains a nearby rectangular building. Exclude distant trees
# and ground; retain upright facade surfaces for the temporal comparison.
candidates=reference[(np.linalg.norm(reference[:,:2],axis=1)<12)&
                     (reference[:,2]>0.5)&(reference[:,2]<6)]
rng=np.random.default_rng(5)
planes=[]
for _ in range(3):
    if len(candidates)<40:break
    best=np.zeros(len(candidates),dtype=bool)
    for _ in range(1500):
        a,b,c=candidates[rng.choice(len(candidates),3,replace=False)]
        normal=np.cross(b-a,c-a)
        norm=np.linalg.norm(normal)
        if norm<1e-8:continue
        normal/=norm
        if abs(normal[2])>0.4:continue
        mask=np.abs((candidates-a)@normal)<0.08
        if mask.sum()>best.sum():best=mask
    if best.sum()<40:break
    points=candidates[best]
    center=points.mean(axis=0)
    _,_,basis=np.linalg.svd(points-center,full_matrices=False)
    normal=basis[2]
    uv=(points-center)@basis[:2].T
    lo,hi=np.percentile(uv,[2,98],axis=0)
    planes.append((center,basis,lo-0.1,hi+0.1))
    candidates=candidates[np.abs((candidates-center)@normal)>0.15]
if len(planes)<2:raise RuntimeError("Insufficient repeatable facade planes in reference snapshot")

report={"method":"Fixed earliest-map facade planes/ROIs; no inter-map alignment; diagnostic only", "snapshots":[]}
fig,axes=plt.subplots(1,len(data),figsize=(5*len(data),6),squeeze=False)
for ax,path,item in zip(axes[0],args.snapshots,data):
    points=item["points"].astype(float)
    statistics=[]
    for index,(center,basis,lo,hi) in enumerate(planes):
        relative=points-center
        uv=relative@basis[:2].T
        distance=relative@basis[2]
        mask=(uv>=lo).all(axis=1)&(uv<=hi).all(axis=1)&(np.abs(distance)<0.8)
        residuals=distance[mask]
        if len(residuals)<20:raise RuntimeError("Insufficient wall support in "+path)
        stats={"plane":index,"points":int(len(residuals)),
               "normal_median_m":float(np.median(residuals)),
               "normal_p90_minus_p10_m":float(np.percentile(residuals,90)-np.percentile(residuals,10))}
        statistics.append(stats)
        print(path,stats,flush=True)
    report["snapshots"].append({"path":path,"map_points":len(points),"walls":statistics})
    facade=points[(points[:,2]>0.5)&(points[:,2]<6)]
    ax.scatter(facade[:,0],facade[:,1],s=0.4,c=facade[:,2],vmin=.5,vmax=6,cmap="viridis",rasterized=True)
    for index,(center,basis,lo,hi) in enumerate(planes):
        ax.text(center[0],center[1],str(index),color="red")
    ax.set_xlim(-3,11);ax.set_ylim(-7,8);ax.set_aspect("equal");ax.grid(alpha=.2)
    ax.set_title("t={:.2f}s; {} points".format(float(item["stamp"])-1583836591.137266,len(points)))
fig.tight_layout()
fig.savefig(args.output+".png",dpi=150)
with open(args.output+".json","w") as output:json.dump(report,output,indent=2)
