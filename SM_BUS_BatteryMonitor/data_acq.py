import serial
data = {}
cell_voltages = []


def conv(hex):
    return chr(int(hex,16))

def extract_data():
    arduino = serial.Serial('COM6', 115200)
    while True:
        dat = arduino.readline().decode('utf-8')
        dat = dat.replace('\r',"").strip()
        # if dat.isnumeric():
        #     print(dat)
        dat = dat.split('\t')
        if dat[0].__contains__('A'):

            if dat[0]=="A0" and dat[0] not in data.keys():
                manf_name = [conv(x) for x in dat[1:]]# map(conv,  dat[1:])
                data["Manufacturer Name"] = "".join(manf_name)
                # print(data["Manufacturer Name"])
            if dat[0]=="A1" and dat[0] not in data.keys():
                data["Device Name"] = "".join([conv(x) for x in dat[1:]])
            if dat[0]=="A2" and dat[0] not in data.keys():
                data["Chemistry"] = "".join([conv(x) for x in dat[1:]])
            if dat[0]=="A3" and dat[0] not in data.keys():
                data["Serial No"] = "".join(dat[1:])

        elif dat[0].__contains__('B'):
            if dat[0]=="B0" and dat[0] not in data.keys():
                data["Design Capacity"] = "".join(dat[1:])
            if dat[0]=="B1" and dat[0] not in data.keys():
                data["Design Voltage"] = "".join(dat[1:])
            if dat[0]=="B2" and dat[0] not in data.keys():
                data["Charge Current"] = "".join(dat[1:])
            if dat[0]=="B3" and dat[0] not in data.keys():
                data["Charge Voltage"] = "".join(dat[1:])


        elif dat[0].__contains__('C'):
            cell_voltages.append("".join(dat[1:]))
            data["Cell voltages"] = cell_voltages


        elif dat[0].__contains__('D'):
            if dat[0]=="D0" and dat[0] not in data.keys():
                tot_vol = "".join(dat[1:])
                data["Total Voltage"] = tot_vol[:2]+ "." + tot_vol[2:]
            if dat[0]=="D1" and dat[0] not in data.keys():
                data["Full Capacity"] = "".join(dat[1:])
            if dat[0]=="D2" and dat[0] not in data.keys():
                data["Temperature"] = "".join(dat[1:])


        if len(data.keys()) == 12:
            arduino.close()
            return data
    # print(cell_voltages)


    # print(dat)
if __name__ == '__main__':
    data = extract_data()
    print(data)