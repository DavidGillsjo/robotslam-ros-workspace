class DataContainers:
    def __init__(self):
        self.containers = {}

    def __iter__(self):
        return iter(self.containers.values())

    def contains(self, ssid):
        return ssid in self.containers

    def get(self, ssid, bssid):
        if ssid not in self.containers:
            self.containers[ssid] = DataContainer(ssid, bssid)
        return self.containers[ssid]

class DataContainer:
    def __init__(self, ssid, bssid):
        self.ssid = ssid
        self.bssid = bssid
        self.x_data = []
        self.y_data = []
        self.line = None

    def put(self, x, y):
        if (len(self.x_data) > 0 and self.x_data[-1] == x):
            self.y_data[-1] = y
        else:
            self.x_data.append(x)
            self.y_data.append(y)

    def get_data(self):
        x_data = self.x_data
        y_data = self.y_data
        #self.clear()
        return (x_data, y_data)

    def clear(self):
        self.x_data = []
        self.y_data = []
