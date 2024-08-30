import json
import struct


def json2bin(dict_data):
    return json.dumps(dict_data).encode("ascii")


def bin2json(dict_bin):
    return json.loads(dict_bin)


def bin2int(int_bin):
    return struct.unpack(">I", int_bin)[0]


def int2bin(integer):
    return struct.pack(">I", integer)
