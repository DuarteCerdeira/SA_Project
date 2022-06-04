#! /usr/bin/env python3
import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd


def read_bag():
    bag = bagreader('2022-05-31-15-10-35.bag')
    print(bag.topic_table)
    laser_msgs = bag.message_by_topic('/scan')
    df_laser = pd.read_csv(laser_msgs)
    print(df_laser)


if __name__ == '__main__':
    read_bag()
