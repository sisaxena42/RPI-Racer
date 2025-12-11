from hokuyolx import HokuyoLX

# Change this if nmap showed a different IP!
LIDAR_IP = "192.168.0.10"
LIDAR_PORT = 10940

laser = HokuyoLX(addr=(LIDAR_IP, LIDAR_PORT))  # uses SCIP 2.x over Ethernet
timestamp, distances = laser.get_dist()  # one full scan

print("Got scan at timestamp:", timestamp)
print("Number of beams:", len(distances))
print("First 10 distances (mm):", distances[:10])
