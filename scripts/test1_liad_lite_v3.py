import Lidar_Lite
L1 = Lidar_Lite.LiDAR_Lite()
L1.connect(1)
b = 1#2.0/(18.0+1.0)
dist = 1;
while(1):
    dist = b*L1.get_distance() + (1-b)*dist
    print(dist)
