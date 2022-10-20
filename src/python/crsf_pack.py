

from CRSF import CRSFWrap
from crc import CrcCalculator, Crc8,Configuration

data=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

def pack1(data):

    result=[0xc8,0x18,0x16]

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
    crc=crc_calculator.calculate_checksum(result[2:])
    print("crc:"+str(crc))
    result.append(crc)
    return result

def pack2(data):
    packet=[0]*26

    packet[0] = 0xc8
    packet[1] = 0x18
    packet[2] = 0x16
    packet[3] = (data[0] & 0x07FF) & 0xff
    packet[4] = ((data[0] & 0x07FF) >> 8 | (data[1] & 0x07FF) << 3) & 0xff
    packet[5] = ((data[1] & 0x07FF) >> 5 | (data[2] & 0x07FF) << 6)& 0xff
    packet[6] = ((data[2] & 0x07FF) >> 2)& 0xff
    packet[7] = ((data[2] & 0x07FF) >> 10 | (data[3] & 0x07FF) << 1)& 0xff
    packet[8] = ((data[3] & 0x07FF) >> 7 | (data[4] & 0x07FF) << 4)& 0xff
    packet[9] = ((data[4] & 0x07FF) >> 4 | (data[5] & 0x07FF) << 7)& 0xff
    packet[10] = ((data[5] & 0x07FF) >> 1)& 0xff
    packet[11] = ((data[5] & 0x07FF) >> 9 | (data[6] & 0x07FF) << 2)& 0xff
    packet[12] = ((data[6] & 0x07FF) >> 6 | (data[7] & 0x07FF) << 5)& 0xff
    packet[13] = ((data[7] & 0x07FF) >> 3)& 0xff
    packet[14] = ((data[8] & 0x07FF))& 0xff
    packet[15] = ((data[8] & 0x07FF) >> 8 | (data[9] & 0x07FF) << 3)& 0xff
    packet[16] = ((data[9] & 0x07FF) >> 5 | (data[10] & 0x07FF) << 6)& 0xff
    packet[17] = ((data[10] & 0x07FF) >> 2)& 0xff
    packet[18] = ((data[10] & 0x07FF) >> 10 | (data[11] & 0x07FF) << 1)& 0xff
    packet[19] = ((data[11] & 0x07FF) >> 7 | (data[12] & 0x07FF) << 4)& 0xff
    packet[20] = ((data[12] & 0x07FF) >> 4 | (data[13] & 0x07FF) << 7)& 0xff
    packet[21] = ((data[13] & 0x07FF) >> 1)& 0xff
    packet[22] = ((data[13] & 0x07FF) >> 9 | (data[14] & 0x07FF) << 2)& 0xff
    packet[23] = ((data[14] & 0x07FF) >> 6 | (data[15] & 0x07FF) << 5)& 0xff
    packet[24] = ((data[15] & 0x07FF) >> 3)& 0xff
    return packet

print(bytes(pack1(data)))
print(bytes(pack2(data)))



