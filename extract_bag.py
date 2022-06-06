from bagpy import bagreader
import pandas as pd

b = bagreader('2022-05-24-06-55-16.bag')

print(b.topic_table)

scan = b.message_by_topic('/scan')

scandf = pd.read_csv(scan)

print(scandf["angle_min"])
print(scandf["angle_max"])
print(scandf["angle_increment"])
