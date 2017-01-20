import scapy.all as sca
import time
import thread
import threading
import signal
import sys
from matplotlib import pyplot as plt
from matplotlib import rcParams
import struct

class ScapyRssi:
    def __init__(self, interface):
        # Radiotap field specification
        self.radiotap_formats = {"TSFT":"Q", "Flags":"B", "Rate":"B",
            "Channel":"HH", "FHSS":"BB", "dBm_AntSignal":"b", "dBm_AntNoise":"b",
            "Lock_Quality":"H", "TX_Attenuation":"H", "dB_TX_Attenuation":"H",
            "dBm_TX_Power":"b", "Antenna":"B",    "dB_AntSignal":"B",
            "dB_AntNoise":"B", "b14":"H", "b15":"B", "b16":"B", "b17":"B", "b18":"B",
            "b19":"BBB", "b20":"LHBB", "b21":"HBBBBBH", "b22":"B", "b23":"B",
            "b24":"B", "b25":"B", "b26":"B", "b27":"B", "b28":"B", "b29":"B",
            "b30":"B", "Ext":"B"}
        # data
        self.data = {}
        self.interface = interface
        self.dataMutex = thread.allocate_lock()
        sca.sniff(iface=self.interface, prn=self.sniff)
        

    def sniff(self, pkt):

        print(pkt.summary())

        addr, rssi = self.parsePacket(pkt)
        if addr is not None:
            with self.dataMutex:
                if addr in self.data.keys():
                    self.data[addr].append(rssi)
                    print(rssi)
                else:
                    self.data[addr] = [rssi]

                #self.data[ssid] = pkt.info
    def parsePacket(self, pkt):
        if pkt.haslayer(sca.Dot11):
            if pkt.addr2 is not None:
                # check available Radiotap fields
                field, val = pkt.getfield_and_val("present")
                names = [field.names[i][0] for i in range(len(field.names)) if (1 << i) & val != 0]
                # check if we measured signal strength
                if "dBm_AntSignal" in names:
                    # decode radiotap header
                    fmt = "<"
                    rssipos = 0
                    for name in names:
                        # some fields consist of more than one value
                        if name == "dBm_AntSignal":
                            # correct for little endian format sign
                            rssipos = len(fmt)-1
                        fmt = fmt + self.radiotap_formats[name]
                    # unfortunately not all platforms work equally well and on my arm
                    # platform notdecoded was padded with a ton of zeros without
                    # indicating more fields in pkt.len and/or padding in pkt.pad
                    decoded = struct.unpack(fmt, pkt.notdecoded[:struct.calcsize(fmt)])
                    return pkt.addr2, decoded[rssipos]
        return None, None
    def plot(self, num):
        plt.clf()
        rcParams["font.family"] = "serif"
        rcParams["xtick.labelsize"] = 8
        rcParams["ytick.labelsize"] = 8
        rcParams["axes.labelsize"] = 8
        rcParams["axes.titlesize"] = 8
        data = {}
        time1 = time.time()
        with self.dataMutex:
            data = dict(self.data)
        nodes = [x[0] for x in sorted([(addr, len(data[addr])) for addr in data.keys()], key=lambda x:x[1], reverse=True)]
        nplots = min(len(nodes), num)
        for i in range(nplots):
            plt.subplot(nplots, 1, i+1)
            plt.title(str(nodes[i]) + ": " 
                + str(len(data[nodes[i]])) + " packets @ " +
                "{0:.2f}".format(len(data[nodes[i]])/(time1-self.time0)) 
                + " packets/sec")
            plt.hist(data[nodes[i]], range=(-100, -20), bins=80)
            plt.gca().set_xlim((-100, -20))
        plt.gcf().set_size_inches((6, 4*nplots))
        plt.savefig("hists.pdf")

if __name__ == "__main__":
    sniffer = ScapyRssi("mon0")
    time.sleep(30)
    sniffer.plot(20)
    print "plotted"
