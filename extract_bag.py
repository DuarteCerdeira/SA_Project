from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt

b = bagreader('2022-05-19-18-59-52.bag')

print(b.topic_table)

pose = b.message_by_topic('/pose')

posedf = pd.read_csv(pose)

print(posedf)

x = posedf["pose.pose.position.x"]
y = posedf["pose.pose.position.y"]

posedf.plot(x='pose.pose.position.x', y='pose.pose.position.y')

plt.show()
