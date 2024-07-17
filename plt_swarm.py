import csv
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression
from collections import namedtuple
import re
import pandas as pd

DW_TIME_TO_MS = (1.0 / 499.2e6 / 128.0) * 1000  # 将dw3000时间戳转换为ms

mylist_sniffer_address=[]
mylist_sniffer_seq=[]
mylist_sniffer_index=[]
mylist_sniffer_rx=[]

off=0

#需要动态修改的地方
#addr在第几列 就将2改成几
#seq sniffer_rx_time 同理
row_sender_addr=2
row_sender_seq=3
row_sender_rx=5
plt_move=5  #移动距离 default=0

address_begin_seq=[-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
AVERAGE_PERIOD=60-0.04 # 平均周期 default=10ms


#选择任意一个已经有的地质作为FIRST_ADDRESS
FIRST_ADDRESS=2

COLOR_LIST=['yellow','pink','orange','red','green',
            'blue','purple','brown','gray','olive',
            'cyan','lime','navy','tan','magenta',
            'gold','silver','violet','indigo','crimson']
def read_data():
    with open(r"/home/luffy/crazyflie/sniffer/my_sniffer/swarm_data3.csv",encoding="utf-8") as F:
        Index=0
        reader=csv.reader(F)
        Header=next(reader)
        for row in reader:
            # if(Index==0):
            #     FIRST_ADDRESS=int(row[row_sender_addr])
            if(int(row[row_sender_addr])==FIRST_ADDRESS):
                Index=Index+1
            mylist_sniffer_address.append(int(row[row_sender_addr]))    
            mylist_sniffer_index.append(Index)
            mylist_sniffer_seq.append(int(row[row_sender_seq]))
            mylist_sniffer_rx.append(float(row[row_sender_rx]))

def plot_period():
    for i in range(len(mylist_sniffer_index)-1):
        plt.plot((mylist_sniffer_rx[i]+plt_move)%AVERAGE_PERIOD,mylist_sniffer_rx[i]/AVERAGE_PERIOD-mylist_sniffer_rx[1]/AVERAGE_PERIOD,'o',color=COLOR_LIST[mylist_sniffer_address[i]],markersize=1)
    
    plt.ylabel("seq")
    plt.title(' multi crazyflie period change in sniffer view ')
    plt.xlim(0,AVERAGE_PERIOD)

    plt.show()
    
if __name__ == '__main__':
    read_data()

    plot_period()