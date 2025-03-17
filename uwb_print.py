import datetime

import usb.util
import struct
import pickle
import pandas as pd


def loss_count(x: pd.DataFrame):
    seq_num = list(x['seq_num'])
    seq_num.sort()
    received_total = len(seq_num)
    seq_span = seq_num[-1] - seq_num[0] + 1
    print("sender: {0}, seq_span: {1}, total_received: {2}, loss_rate: {3}".format(x.iloc[0]['sender_addr'], seq_span,
                                                                                   received_total,
                                                                                   received_total / seq_span))


if __name__ == '__main__':
    vendor_id = 0x0483
    product_id = 0x5740

    dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

    if dev is None:
        raise ValueError("cannot find usb device")

    dev.set_configuration()

    endpoint = dev[0][(0, 0)][0]

    log_data = []

    try:
        while True:
            meta = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
            try:
                magic, sender_addr, seq_num, msg_len, sniffer_rx_time = struct.unpack("<IHHHQ", meta)
                meta_dict = {'magic': magic, 'sender_addr': sender_addr, 'seq_num': seq_num, 'msg_len': msg_len,
                             'sniffer_rx_time': sniffer_rx_time}
                # print(meta_dict)
            except struct.error:
                pass
            else:
                if magic == 0xBB88:
                    meta_dict['bin_data'] = dev.read(endpoint.bEndpointAddress, msg_len)
                    # 将 array.array 转换为字节字符串
                    bin_data_bytes = bytes(meta_dict['bin_data'])
                    # 解码为 ASCII
                    ascii_string = bin_data_bytes.decode('ascii', errors='ignore')  # 'ignore'会忽略不可解码的字符
                    print("字符：", ascii_string)
                    log_data.append(meta_dict)

    except KeyboardInterrupt:
        pass
    finally:
        if len(log_data) > 0:
            with open('./data/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.pkl', 'wb') as file:
                pickle.dump(log_data, file)
        usb.util.release_interface(dev, 0)
        usb.util.dispose_resources(dev)

    # with open('filename', 'rb') as file:
    #     data = pickle.load(file)
    # df = pd.DataFrame(data)
    # df.groupby('sender_addr').apply(loss_count)
