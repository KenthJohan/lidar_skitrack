# Pipe lidar points to detection
# Detection sends graphics to address. mg.exe can receive that data.
recorder/ce30_recorder -s | detection/detection_file -a"tcp://192.168.1.176:9002" -i
