import ctypes
from ctypes import c_uint8,c_uint32
from crc import CrcCalculator, Crc8,Configuration
from serial import Serial
import time

# pip install crc  first




class CRSFData(ctypes.LittleEndianStructure):
    _fields_ = [ ("data"+str(i), c_uint32,11) for i in range(16)]

class CRSFBytes(ctypes.Union):
    _fields_=[
        ("data",CRSFData),
    ("bytes",c_uint8*22)]


class CRSFWrap():
    def __init__(self) -> None:
        self.index=0
        
        self.raw:CRSFBytes=CRSFBytes()
        self.data:CRSFData=self.raw.data
        self.bytes=self.raw.bytes

    def append(self,item):
        #attr_name="data"+str(self.index)
        #self.data.__setattr__(attr_name,item)
        #print(self.data.__getattribute__(attr_name))
        self[self.index]=item
        self.index+=1

    def __getitem__(self, y):
        attr_name="data"+str(y)
        return self.data.__getattribute__(attr_name)
    
    def __setitem__(self,index,value):
        attr_name="data"+str(index)
        self.data.__setattr__(attr_name,value)
    
    def pack_wrong(self):
        header=[0xc8,0x18]
        type_bytes=(0x16).to_bytes(1,byteorder='little')

        crc_bytes=type_bytes+bytes(self.bytes)
        print("crc_bytes:"+ str(crc_bytes))


        crc_config=Configuration(8,0xd5)
        crc_calculator = CrcCalculator(crc_config)
        crc=crc_calculator.calculate_checksum(crc_bytes).to_bytes(1,byteorder='little')
        print("crc:"+str(crc))

        final=bytes(header) + crc_bytes + crc

        print(final)

        return final
    
    def pack(self):
        data=self.data
        result=[]
        start_bit_index=0
        byte=0

        for i in data:
            temp=i
            num = 11
            while num!=0:
                data1=temp & ((1<<(8-start_bit_index))-1)
                pack_bytes= min(num,8-start_bit_index)
                num -= pack_bytes

                byte = (data1 << start_bit_index) | byte

                temp = temp >> pack_bytes
                start_bit_index += pack_bytes

                if( start_bit_index == 8):
                    result.append(byte)
                    byte=0
                    start_bit_index=0
    
        crc_config=Configuration(8,0xd5)
        crc_calculator = CrcCalculator(crc_config)
        crc=crc_calculator.calculate_checksum(result[2:]).to_bytes(1,byteorder='little')
        result.append(crc)
        return result

    @staticmethod
    def from_raw_chn(string):
        # E0 03 1F 2B C0 F7 8B 5F FC E2 17 E5 2B 5F F9 CA 07 00 00 44 3C E2
        # should be 22 bytes
        arr=string.split(" ")
        result=CRSFWrap()
        cnt=0
        if(len(arr)!=22):
            print("err length for raw channels!")
        for i in arr:
            int_num=int(i,16)
            result.raw.bytes[cnt]=int_num
            cnt+=1
        return result

'''        
a=CRSFWrap()
a.append(8)
a.append(9)

print(bytes(a.raw.bytes))
print(a.raw.bytes[0])
print(a.raw.bytes[1])

a.pack()
'''


b=CRSFWrap.from_raw_chn("E0 03 1F 2B C0 F7 8B 5F FC E2 17 E5 2B 5F F9 CA 07 00 00 44 3C E2")
ddd=b.pack_wrong()

serial=Serial(port="com3",baudrate=460800)

while True:
    serial.write(ddd)
    time.sleep(0.1)
    #if serial.in_waiting>0:
    #    data=serial.read_all()
    #    print(data)
    #time.sleep(0.1)