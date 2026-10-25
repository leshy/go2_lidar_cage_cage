# Voxel-remesh menagerie base_4 (head region) into a watertight shell, in the lidar frame (mm).
import trimesh, numpy as np, sys
th=2.8782; o=np.array([289.45,0,-46.825])
R=np.array([[np.cos(th),0,np.sin(th)],[0,1,0],[-np.sin(th),0,np.cos(th)]])
m=trimesh.load("ref/base_4.obj",force="mesh"); m.vertices=(m.vertices*1000-o)@R
c=m.triangles_center; keep=(c[:,0]>-75)&(c[:,0]<110)&(np.abs(c[:,1])<75)&(c[:,2]>-35)
m=m.submesh([np.nonzero(keep)[0]],append=True)
v=m.voxelized(pitch=1.0)
s=v.marching_cubes; s.apply_transform(v.transform)
s=trimesh.smoothing.filter_taubin(s,iterations=5)

print("watertight",s.is_watertight,len(s.faces),s.bounds.round(1).tolist())
s.export("ref/head_shell_lidar_frame.stl")
