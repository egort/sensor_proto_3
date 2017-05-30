#!/usr/bin/env python3
"""
 *  This file is part of plainRFM69.
 *  Copyright (c) 2014, Ivor Wanders
 *  MIT License, see the LICENSE.md file in the root folder.
"""


import crcmod
from Crypto.Cipher import AES

"""
    This script does some analysis on the packets produced by the RFM69 radio
    modules. It investigates the cipher mode and the CRC polynomial.
"""


print("Decoding a few packets using ECB cipher mode.\n")
packets=["8E7760F05C573E15AA5BE39470BE70CC202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
         "E7D3AF2F7EBE569B7EE2F2F3EE825F2E202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
         "ED660A0682E94BBBBB98D9E84B7EBDE3202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
         "B720DD819F49264684815C4767BC5A8B202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "5ACE8F6CC4710212D2CB294792BF1D7E202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "B26162233AFC2D47ADDFB4B92D0697C2202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "DBC3A0068B8ACAB1867B6F8E897F8DA0202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "D8AD1CA54F5F086861C5C9808E020903202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "BA9EDEA3D76ED5A123A1314F86296BCB202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "A213715080D4BE92C80EAF1ADF1C8EF0202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "0216DC6EA3C50DF948AB69A0A578A9BC202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",
        "23E0D6A53E72AFA05C7E0AA438A10080202C11692E65C99BCB7BA90B1B61524A02A0ECCC1F2BC60C836CB312E81B3A3F",

        ]

aes_ECB_zeros = AES.new(bytes([0 for i in range(0,16)]), AES.MODE_ECB)
packetbytes = [bytes([int(j[i:i+2],16) for i in range(0,len(j), 2)]) for j in packets]
# print(packetbytes)
for j in packetbytes:
    print(aes_ECB_zeros.decrypt(j))




# Sent with:
# uint8_t key[16] = {146, 48, 0, 16, 31, 203, 208, 65, 31, 10, 94, 64, 8, 198, 226, 121};
# rfm->setPacketConfig1(RFM69_PACKET_CONFIG_CRC_ON);
# rfm->setPacketConfig2(11, true, true, true);
# rfm->setPayloadLength(20);
# Received as:
# 0xAA0000004445464748494A4B4C4D4E4F505152530000000000
# 0xAD0000004445464748494A4B4C4D4E4F505152530000000000

# Recieved with CRC off, AES off, payloadlength of 34.
# 0xA02AEBADF27DBBB36F3928BD53A3BC87AD7159E950A85F4ABF59F043828932B7226E
# 0x2B240611993CEB856A07FD353C940ACAAD7159E950A85F4ABF59F043828932B73A22
# 0x2475FEF5D93D2EE636B43584DEDCF622AD7159E950A85F4ABF59F043828932B7A639
# 0x27D3806ED8A8BB63BD700B9FAE8B64C9AD7159E950A85F4ABF59F043828932B782D4
# 0x652729B2F2A75C1279AF2833417DFCF5AD7159E950A85F4ABF59F043828932B76DA1


secondblob= """0x40C3D18D9DD0B5AA282163095BCAA2A3AD7159E950A85F4ABF59F043828932B71FCE
0x85096C2B74B868AB0028B8EB1C5F32DFAD7159E950A85F4ABF59F043828932B74531
0x775CA8AC1172E7BE0087620AB85A3FA5AD7159E950A85F4ABF59F043828932B79BC8
0x2171F599317F14647152AF7575878392AD7159E950A85F4ABF59F043828932B719EF
0x9EEA1DEDD22250EBE86A9E76C8FD09E9AD7159E950A85F4ABF59F043828932B7EBA2
0x69B8EE198B6D1D86F2C325C99433365BAD7159E950A85F4ABF59F043828932B70D11
0x0B4CF2CAC1C54C7FBC74166E56DDB8BEAD7159E950A85F4ABF59F043828932B79FD5
0xEAC5E015D784F5EBD52288E29DE829E1AD7159E950A85F4ABF59F043828932B74D6F
0x04C6F98A3AE2F734F04B5AD6BCD331B7AD7159E950A85F4ABF59F043828932B7F59C
0xEC8BC1DF88627E05CA4E7A8E61FF66A4AD7159E950A85F4ABF59F043828932B73DAC
0xCBE28965605C1DCCC49DE89E70740AC4AD7159E950A85F4ABF59F043828932B7E348
0xDA0F57C322198373F8B8D5F1C2092973AD7159E950A85F4ABF59F043828932B75675
0x712E58D47D65099DA1284FD2468A2D92AD7159E950A85F4ABF59F043828932B7636B
0xEB9749AF00AD035E9F5FD10192B0BE8DAD7159E950A85F4ABF59F043828932B77780
0x78A93AE0086ACAE9C40EB21BC1ED3780AD7159E950A85F4ABF59F043828932B76758
0x5C5F1708EAFFB0AE5C099D80B8E4EA1AAD7159E950A85F4ABF59F043828932B77233
0x3DCF439921381E5759032FDC4A1A5256AD7159E950A85F4ABF59F043828932B7F9C2
0x7AB451B10DFFF162876E8A5CC7A43624AD7159E950A85F4ABF59F043828932B729A3
0x200F94DFABB83D2AD9257B89861E0838AD7159E950A85F4ABF59F043828932B70ABB
0x2A1C2DE6E3E61B18F4BE6F2A8735B994AD7159E950A85F4ABF59F043828932B71458
0xED7A09F2F4855703ED61F2BF900FB05AAD7159E950A85F4ABF59F043828932B7965E
0xAD86E27BFCC2ADB7163D8EC3B1F2C7EFAD7159E950A85F4ABF59F043828932B7F02A
0xE2DF08D40982ED92F3B7240165ACFC7CAD7159E950A85F4ABF59F043828932B76F0D
0xE48A637BB4FC8163CDAB64B31CB91D94AD7159E950A85F4ABF59F043828932B7646B
0xBFB6B5347425C5E0878C70AB4034C51BAD7159E950A85F4ABF59F043828932B71CF6
0x3AF7D0B56189B60990732EE7CC0AA3B7AD7159E950A85F4ABF59F043828932B7FC0D
0x99CB3B7C809BF435337861DA21A409A9AD7159E950A85F4ABF59F043828932B76515
0xB852EC76F7E645387224FA49A55A681AAD7159E950A85F4ABF59F043828932B768BC
0x2BFEE1521309749AFA635366CA2A959CAD7159E950A85F4ABF59F043828932B7F8D5
0x9A5CBD91C84EBE906EF16A818FCCB7F4AD7159E950A85F4ABF59F043828932B73458
0x4BEEC7794EF3E72DD8432524E7621428AD7159E950A85F4ABF59F043828932B78ED5
0x57E40429BD34F151D9D7A8A5758F7903AD7159E950A85F4ABF59F043828932B70038
0xD45FDF85ACB352A02275448281DEF736AD7159E950A85F4ABF59F043828932B761CB
0x759BFBB7F8E04F335B285DA414F69D6DAD7159E950A85F4ABF59F043828932B7323C
0x0F9F13D6B72AF71E9B34A28671DB12B8AD7159E950A85F4ABF59F043828932B711D1
0x9382E8DFC3803F9F1D3CC169EFC2D6FAAD7159E950A85F4ABF59F043828932B7967E
0xA3D0603CFBE16DBC9E8EB9F625BF0014AD7159E950A85F4ABF59F043828932B72C7A"""


print("\n\nCalculating CRC XOR and INIT parameters with several packets.\n")
packets=[
"A02AEBADF27DBBB36F3928BD53A3BC87AD7159E950A85F4ABF59F043828932B7226E",
"2B240611993CEB856A07FD353C940ACAAD7159E950A85F4ABF59F043828932B73A22",
"2475FEF5D93D2EE636B43584DEDCF622AD7159E950A85F4ABF59F043828932B7A639",
"27D3806ED8A8BB63BD700B9FAE8B64C9AD7159E950A85F4ABF59F043828932B782D4",
"652729B2F2A75C1279AF2833417DFCF5AD7159E950A85F4ABF59F043828932B76DA1"]
packetbytes = [bytes([int(j[i:i+2],16) for i in range(0,len(j), 2)]) for j in packets]
# print(packetbytes)
crcbytes = [int(a[-4:],16) for a in packets]



# packetbytes = [bytes([int(j[i+2:i+2+2],16) for i in range(0,len(j)-2, 2)]) for j in secondblob.split("\n")]
# crcbytes = [int(a[-4:],16) for a in secondblob.split("\n")]

# print(crcbytes)
payload = [a[0:-2] for a in packetbytes]
# print(payload)

isDone = False
for init in range(0,0xFFFF):
    for xor in range(0,0xFFFF):
        A = crcmod.mkCrcFun(0x11021, initCrc=init, rev=False, xorOut=xor)
        res = [a for a in list(map(A, payload))]
        # print(crcbytes)
        # print(res1)
        z = [0 if res[i] == crcbytes[i] else 1 for i in range(0,len(res))]
        if (sum(z) == 0):
            print(res)
            print(crcbytes)
            print("Init: {}".format(init))
            print("xor: {}".format(xor))
            isDone = True
            break
            # sys.exit(0)
    if (isDone):
        break

#resulted in xor=1272, init=0, no reverse
print("Verifying the calculated parameters on another packetcapture.")
#verify that on the new blobl.
packetbytes = [bytes([int(j[i+2:i+2+2],16) for i in range(0,len(j)-2, 2)]) for j in secondblob.split("\n")]
crcbytes = [int(a[-4:],16) for a in secondblob.split("\n")]
payload = [a[0:-2] for a in packetbytes]
# print(crcbytes)
A = crcmod.mkCrcFun(0x11021, initCrc=0, rev=False, xorOut=1272)
res = list(map(A, payload))

z = [0 if res[i] == crcbytes[i] else 1 for i in range(0,len(res))]
print(res[0:7])
print(crcbytes[0:7])
if (sum(z) == 0):
    print("Yes, correct poly.")
    print("crcmod.mkCrcFun(0x11021, initCrc={:d}, rev=False, xorOut={:d})".format(init, xor))


# thing = AES.new(bytes([146, 48, 0, 16, 31, 203, 208, 65, 31, 10, 94, 64, 8, 198, 226, 121]), AES.MODE_ECB)
# decrypted = [thing.decrypt(j[0:16]) + thing.decrypt(j[16:32]) for j in packetbytes]


print("\n\nDetermining how AES cipher is handled.\n")

m ="""
    Sent with variable length, no addressing, variable length taken to be 5. AES (0)
    Received with fixed length 17, no addressing. AES disabled.

    CRC is removed by Rx phase, so that's missing.
"""
print(m)

# print("\nVariable length, no addressing\n");
testblob = """0x05E594B1DA6CD6DD582B92E6C3FB2D0BBF
0x051C485121872B1EA87514326714F73197
0x0546AFBACAC815CC0DF7DC59F0DF5E93AC
0x05B119A2572AE34141F798F8D567157379
0x0570F76CCCFE3B5D83666736587C00C7CF
0x05B62024068A1DBB90B2BC58C4DBCBAB4F
0x055563A1E1193DEE76BFAB68AE5E1E0235
0x05ED021D57E52FC8071B37BCFD3B17487C
0x057656A68E421018542A9E1F55096FE4A8
0x058849BF9CD25613EA53018F8E8C651317
0x05102567E9863433712831B10AA73A85B2
0x05FE36E1A813EE9AB12D6BFB3042AA5D59
0x05F909A334C9C092692C92FD55312389A9
0x05641EB053DCEE83B8C4809025790B5261
0x05FAE6B432A382CAC15D1295CA80EB70E7
0x0566AC800895885E97B03A8A59F25200C1
0x059CD45E03A5973D57D4FEBFBA030359D5"""


print("Packetbytes:")
packetbytes = [bytes([int(j[i+2:i+2+2],16) for i in range(0,len(j)-2, 2)]) for j in testblob.split("\n")]
crcbytes = [int(a[-4:],16) for a in testblob.split("\n")]


aes_ECB_zeros = AES.new(bytes([0 for i in range(0,16)]), AES.MODE_ECB)



for j in packetbytes[0:5]:
    print("Cipher: " + str(j))
for j in packetbytes[0:5]:
    plaintext = aes_ECB_zeros.decrypt(j[1:])
    print("Plain: " + str(plaintext))


"""
Results in:
b'\xad\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
b'\xae\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
b'\xaf\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

"""

m="""
    So, as the datasheet states, the length field is not encrypted.
    Not described is that the message is zero-padded to 16 bytes AES payload.
    Which is necessary for the block cipher to work.
"""
print(m)

m="""
    Sent with variable length, with addressing, variable length set to 6, AES(0)
    Received with fixed length 18, addressing, AES disabled
"""

print("With variable length and adressing")
print(m)

testblob="""0x06ABDBA03075EFA0410C3B7C4206112BB309
0x06AB94E0A6F6B37C4489D484FA8343198406
0x06AB1F2FF5A442F4BB89C420F2D577E2DB44
0x06AB01B88ACE401B5DD17CD96D743632C667
0x06AB4AFD493D859581C4942387991B50768B
0x06AB82228277F697B4D512C17A78A8C42BDA
0x06AB76645F0D6329639DA8F675BB543A2591
0x06AB31794A0E916248A2699C8040D89E58FD
0x06AB10BDE17028E8AAAC9E39C0E8D163863F"""


packetbytes = [bytes([int(j[i+2+2:i+2+2+2],16) for i in range(0,len(j)-4, 2)]) for j in testblob.split("\n")]


"""
b'.\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
b'/\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
b'0\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
b'1\x01\x02\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

"""

# thing = AES.new(bytes([0 for i in range(0,16)]), AES.MODE_ECB)

for j in packetbytes[0:3]:
    print("Cipher: " + str(j))
for j in packetbytes[0:3]:
    plaintext = aes_ECB_zeros.decrypt(j[1:])
    print("Plain: " + str(plaintext))

m="""
    So, if AES is used, the data starting after the possible addressing byte or
    length and address byte is zero padded to obtain blocks of 16 bytes long.
    The length byte is not increased, but the actual data transmitted over the 
    air is.
    Transmitting less than 16 bytes does not result in faster transmissions.
"""
print(m)