import pickle
import struct
import pandas as pd
import numpy as np
import sys


def unpack_bindata(row: pd.DataFrame):
    print(len(row['bin_data']))
    if ((len(row['bin_data'])- 84 ) %13 ==0):
        print(row['seq_num'], end=':')
        format_string = '<HHQHQHQHQHQHhhhfH?BHHfff'
        header = struct.unpack(format_string, row['bin_data'][0:84])
        print(header, end='--')
        print('')
        format_string = '<BHQH'
        iter_items = struct.iter_unpack(format_string, row['bin_data'][84:])
        for item in iter_items:
            print(item, end=',')
            # item[0]是编号,item[1]是时s间戳，item[2]是序号
            print('++++')
        print('---')



with open('2024-07-18-22-17-41-v5.pkl', 'rb') as file:
    data = pickle.load(file)

data = pd.DataFrame(data)

data.apply(unpack_bindata, axis=1)