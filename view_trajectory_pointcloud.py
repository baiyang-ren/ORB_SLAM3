import numpy as np                                                                          
import matplotlib.pyplot as plt                                                             
from mpl_toolkits.mplot3d import Axes3D                                                     
                                                                                            
# Load trajectory                                                                           
traj = np.loadtxt('/home/cv/cuesinc-AI/ORB_SLAM3/CameraTrajectory.txt')                     
                                                                                            
# Load point cloud                                                                          
with open('/home/cv/cuesinc-AI/ORB_SLAM3/PointCloud.ply', 'r') as f:                        
    lines = f.readlines()                                                                   
for i, line in enumerate(lines):                                                            
    if 'end_header' in line:                                                                
        data_start = i + 1                                                                  
        break                                                                               
points = np.array([[float(p) for p in l.split()] for l in lines[data_start:] if             
len(l.split())==3])                                                                         
                                                                                            
fig = plt.figure(figsize=(14, 10))                                                          
ax = fig.add_subplot(111, projection='3d')                                                  
                                                                                            
# Plot subsampled points                                                                    
idx = np.random.choice(len(points), min(5000, len(points)), replace=False)                  
ax.scatter(points[idx,0], points[idx,1], points[idx,2], s=0.3, c='gray', alpha=0.5,         
label='Map')                                                                                
                                                                                            
# Plot trajectory                                                                           
ax.plot(traj[:,1], traj[:,2], traj[:,3], 'r-', linewidth=2, label='Camera Path')            
                                                                                            
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')                                  
ax.legend()                                                                                 
plt.savefig('/home/cv/cuesinc-AI/ORB_SLAM3/map_with_trajectory.png', dpi=150)               
plt.show()