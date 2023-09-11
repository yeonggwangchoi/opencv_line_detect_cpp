import os

path = '/home/fc/catkin_ws/src/gm/opencv_line_detect_cpp/src/data/1'
file_names = os.listdir(path)

i =1

for file_name in file_names:
    os.rename(os.path.join(path, file_name), os.path.join(path, str(i)+'.jpg'))
    i += 1