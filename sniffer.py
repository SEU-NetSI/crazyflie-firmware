import datetime

import usb.util
import struct
import pickle

if __name__ == '__main__':
    vendor_id = 0x0483
    product_id = 0x5740

    dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

    if dev is None:
        raise ValueError("cannot find usb device")

    dev.set_configuration()

    endpoint = dev[0][(0, 0)][0]
    print(endpoint)

    log_data = []

    try:
        while True:
            meta = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
            try:
                magic, sender_addr, seq_num, msg_len, sniffer_rx_time = struct.unpack("<IHHHQ", meta)
                meta_dict = {'magic': magic, 'sender_addr': sender_addr, 'seq_num': seq_num, 'msg_len': msg_len,
                             'sniffer_rx_time': sniffer_rx_time}
                print(meta_dict)
            except struct.error:
                pass
            else:
                if magic == 0xBB88:
                    # TODO:注意msg_len不是20
                    meta_dict['bin_data'] = dev.read(endpoint.bEndpointAddress, msg_len)
                    log_data.append(meta_dict)
    except KeyboardInterrupt:
        pass
    finally:
        if len(log_data) > 0:
            with open('./newdata/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.pkl', 'wb') as file:
                pickle.dump(log_data, file)
        usb.util.release_interface(dev, 0)
        usb.util.dispose_resources(dev)