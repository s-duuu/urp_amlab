import os
import pandas as pd
import csv

os.chdir('/home/heven/CoDeep_ws/src/cm_Camera_LiDAR_Fusion/src/csv/practice')

f = open('practice1.csv', 'w')

a = [[1,1], [2,3]]
b = [[1,4], [2,7]]
dict = {1: a, 2: b}

dict[1].append([3, 4])

writer = csv.writer(f)
writer.writerows(dict[1])

f.close()