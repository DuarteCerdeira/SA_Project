from bagpy import bagreader
import pandas as pd

b = bagreader('2022-05-24-06-55-16.bag')

print(b.topic_table)

scan = b.message_by_topic('/scan')

scandf = pd.read_csv(scan)

print(scandf["angle_min"])

str1 = ["ranges_" + str(i) for i in range(725)]

laser_data = [scandf[str1]]

print(laser_data)
