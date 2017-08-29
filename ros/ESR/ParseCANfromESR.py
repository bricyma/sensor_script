import matplotlib.pyplot as plt


idBase = 1280

class ParseESR:
    def __init__(self):
        self.file = "esr.txt"
        self.id = []  # 1344
        self.frame_count = []
        self.rx_count = []  # CAN Rx Event
        self.time = []  # time:1726520
        self.diff_time = []  # time2 - time1
        self.data = {}
        for i in range(0, 64):
            self.data[str(i)] = {'id': i}
        self.msgToFunc = {
            1280: self.track_msg,
            1281: self.track_msg,
            1282: self.track_msg,
            1283: self.track_msg,
            1284: self.track_msg,
            1285: self.track_msg,
            1286: self.track_msg,
            1287: self.track_msg,
            1288: self.track_msg,
            1289: self.track_msg,
            1290: self.track_msg,
            1291: self.track_msg,
            1292: self.track_msg,
            1293: self.track_msg,
            1294: self.track_msg,
            1295: self.track_msg,
            1296: self.track_msg,
            1297: self.track_msg,
            1298: self.track_msg,
            1299: self.track_msg,
            1300: self.track_msg,
            1301: self.track_msg,
            1302: self.track_msg,
            1303: self.track_msg,
            1304: self.track_msg,
            1305: self.track_msg,
            1306: self.track_msg,
            1307: self.track_msg,
            1308: self.track_msg,
            1309: self.track_msg,
            1310: self.track_msg,
            1311: self.track_msg,
            1312: self.track_msg,
            1313: self.track_msg,
            1314: self.track_msg,
            1315: self.track_msg,
            1316: self.track_msg,
            1317: self.track_msg,
            1318: self.track_msg,
            1319: self.track_msg,
            1320: self.track_msg,
            1321: self.track_msg,
            1322: self.track_msg,
            1323: self.track_msg,
            1324: self.track_msg,
            1325: self.track_msg,
            1326: self.track_msg,
            1327: self.track_msg,
            1328: self.track_msg,
            1329: self.track_msg,
            1330: self.track_msg,
            1331: self.track_msg,
            1332: self.track_msg,
            1333: self.track_msg,
            1334: self.track_msg,
            1335: self.track_msg,
            1336: self.track_msg,
            1337: self.track_msg,
            1338: self.track_msg,
            1339: self.track_msg,
            1340: self.track_msg,
            1341: self.track_msg,
            1342: self.track_msg,
            1343: self.track_msg,
            1344: self.track_status_msg,
        }

    def parse(self):
        count = 0
        with open(self.file) as f:
            rx_c = 0
            for line in f:
                pos = line.find("id:")
                pos_time = line.find("time:")
                if line.strip() == 'CAN Rx Event':
                    rx_c += 1
                if pos != -1:
                    cur_id = int(line[pos+3:pos+7])
                    if cur_id == 1248:
                        # time:1725201
                        self.time.append(int(line[pos_time + 5:pos_time + 12]))
                        if len(self.time):
                            self.diff_time.append(self.time[len(self.time)-1] - self.time[len(self.time)-2])
                        self.rx_count.append(rx_c)
                        rx_c = 0
                    self.id.append(int(line[pos+3:pos+7]))

                    # parse can frame
                    msgId = cur_id
                    if msgId in self.msgToFunc:
                        msg = self.line2msg(line)
                        if msg is None:
                            continue
                        self.msgToFunc[msgId](msgId, msg)

                # avoid parsing all the information in a large file
                count += 1
                if count > 10000:
                    break

    def line2msg(self, line):
        # data: 15 0a 0a 0a 15 0c 1f 0a flags:0x2 time:1725120
        pos_data = line.find('data')
        pos_flag = line.find('flags')
        if pos_flag == -1:
            return
        data_str = line[pos_data + 6:pos_data + 29]
        data_arr = data_str.split(' ')
        msg = []
        for item in data_arr:
            if item != '':
                msg.append(int(item, 16))

        return msg

    def track_msg(self, msgId, msg):
        if msg == None:
            return
        # """ message ID 500-53F or 1280-1343 """
        track_id = str(msgId-idBase)
        status = ((msg[1] & 0xE0) >> 5)
        if (status < 2 or status > 3): ### Updated target ?
            return
        print msg
        self.data[track_id]["track_oncoming"] = (msg[0] & 0x01)
        self.data[track_id]["track_group_changed"] = ((msg[0] & 0x02) >> 1)
        self.data[track_id]["track_lat_rate"] = ((msg[0] & 0xFC) >> 2)
        self.data[track_id]["track_status"] = ((msg[1] & 0xE0) >> 5)
        self.data[track_id]["track_angle"] = self.parse_track_angle(((msg[1] & 0x1F) << 5) | ((msg[2] & 0xF8) >> 3)) # Spans multiple bytes
        self.data[track_id]["track_range"] = self.parse_track_range(((msg[2] & 0x07) << 8) | msg[3]) # Spans multiple bytes
        self.data[track_id]["track_bridge"] = ((msg[4] & 0x80) >> 7)
        self.data[track_id]["track_rolling_count"] = ((msg[4] & 0x40) >> 6)
        self.data[track_id]["track_width"] = self.parse_track_width((msg[4] & 0x3C) >> 2)
        self.data[track_id]["track_range_accel"] = self.parse_track_range_accel(((msg[4] & 0x03) << 8) | msg[5]) # Spans multiple bytes
        self.data[track_id]["track_med_range_mode"] = ((msg[6] & 0xC0) >> 6)
        self.data[track_id]["track_range_rate"] = self.parse_track_range_rate(((msg[6] & 0x3F) << 8) | msg[7]) # Spans multiple bytes



    def track_status_msg(self, msgId, msg):
        # parse track_status_msg id: 1344
        """
        Moving Flag, 0 = stationary / 1 = moving
        Fast Movable Flag, 0 = not movable / 1 = movable
        Slow Movable Flag, 0 = not movable / 1 = movable
        Track Power (Track amplitude)
        This amplitude has an offset of -10dB. For example, CAN_TX_TRACK_POWER of 31 = 21dB
        set at 31 if > 21dB
        set at 0 if < -10dB
        """
        # """ message ID x540 or 1344 """
        for i in range(1, 8):
            # 0 = Group 1
            group_id = msg[0] & 0x0F
            track_id = group_id * 7 + i
            if track_id >= 64:
                break
            track_id = str(track_id)
            self.data[track_id]["track_movable_fast"] = ((msg[i] & 0x80) >> 7)
            self.data[track_id]["track_movable_slow"] = ((msg[i] & 0x40) >> 6)
            self.data[track_id]["track_moving"] = ((msg[i] & 0x20) >> 5)
            self.data[track_id]["track_power"] = self.parse_track_power(msg[i] & 0x1F)

    #########################################################################
    def parse_track_width(self, binary_format):
        """
        Width set at 7.5 if > 7.5
        Signed +
        Bits: 4
        Scale: 0.5
        Range: 0 to 7.5
        Unit: m
        """
        return self.parse_track_binary(binary_format, False, 4, 0.5)

    def parse_track_power(self, binary_format):
        """
        Track Power (Track amplitude)
        This amplitude has an offset of -10dB. For example, CAN_TX_TRACK_POWER of 31 = 21dB
        set at 31 if > 21dB
        set at 0 if < -10dB
        Signed (+/-)
        Bits: 5
        Scale: 1
        Range: -10 to 21
        Unit: dB
        """
        return self.parse_track_binary(binary_format, False, 5, 1) - 10

    def parse_track_binary(self, binary_format, signed, bits, scale):
        '''
        Util. Max bits 16 in document.
        '''
        if (signed):
            if (binary_format >> (bits - 1) == 1):
                sign = -1
                decimal_format = ((1 << bits) - binary_format)

            else:
                sign = 1
                decimal_format = int(binary_format)

            decimal_format = sign * decimal_format * scale
            return decimal_format
        else:
            return int(binary_format) * scale
    def parse_track_angle(self, binary_format):
        '''
        CAN_TX_TRACK_ANGLE
        Azimuth
            0 = toward front of vehicle parallel to vehicle centerline
            (+) = clockwise (Delphi/Input).  (-) = clockwise(AS/output)
            set at 51.1 if > 51.1
            set at -51.2 if < -51.2
        Signed (+/-)
        Bits: 10
        Scale: 0.1
        Range: -51.2 to 51.1
        Unit: deg
        Default: 0
        '''
        decimal_format = -1 * self.parse_track_binary(binary_format, True, 10, 0.1)
        return decimal_format

    def parse_track_range(self, binary_format):
        '''
        CAN_TX_TRACK_RANGE
        Range
            (+) = away from sensor
            set at 204.7 if > 204.7
        Unsigned (+)
        Bits: 11
        Scale: 0.1
        Range: 0 to 204.7
        Unit: m
        Default: 200
        '''
        return self.parse_track_binary(binary_format, False, 11, 0.1)

    def parse_track_range_accel(self, binary_format):
        '''
        CAN_TX_TRACK_RANGE_ACCEL
        Range Acceleration
            (+) = away from sensor
            set at 25.55 if  > 25.55
            set at -25.6 if < -25.6
        Signed (+/-)
        Bits: 10
        Scale: 0.05
        Range: -25.6 to 25.55
        Unit: m/s/s
        Default: 0
        '''
        return self.parse_track_binary(binary_format, True, 10, 0.05)

    def parse_track_range_rate(self, binary_format):
        '''
        CAN_TX_TRACK_RANGE_RATE
        Range Rate
            (+) = away from sensor
            set at 81.91 if > 81.91
            set at -81.92 if < -81.92
        Signed (+/-)
        Bits: 14
        Scale: 0.01
        Range: -81.92 to 81.91
        Unit: m/s
        Default: 81.91
        '''
        return self.parse_track_binary(binary_format, True, 14, 0.01)

    def plot(self):
        plt.subplot(311)
        plt.plot(self.id, 'r')
        plt.subplot(312)
        plt.plot(self.frame_count, 'r')
        plt.subplot(313)
        plt.plot(self.rx_count, 'r')
        plt.show()


if __name__ == '__main__':
    _esr = ParseESR()
    _esr.parse()
    # _esr.plot()
    # for item in _esr.data:
    #     print _esr.data[item]
