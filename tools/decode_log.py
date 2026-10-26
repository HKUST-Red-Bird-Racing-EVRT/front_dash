"""Decode a dash LOGnnn.BIN to CSV.

Usage: python decode_log.py LOG000.BIN > out.csv
Run with no arguments for a self-check.
"""
import struct
import sys

REC = struct.Struct("<IH8s")  # millis, can_id | bus << 12, data


def decode(raw):
    """Yield (ms, id, data) per record, dropping a trailing partial record."""
    n = len(raw) // REC.size
    for ms, cid, data in REC.iter_unpack(raw[: n * REC.size]):
        yield ms, cid, data


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raw = struct.pack("<IH8s", 1234, 0x701, bytes(range(8))) + b"\x01\x02"
        assert list(decode(raw)) == [(1234, 0x701, bytes(range(8)))]
        print("self-check ok")
        sys.exit()
    print("ms,bus,id," + ",".join(f"d{i}" for i in range(8)))
    for ms, cid, data in decode(open(sys.argv[1], "rb").read()):
        print(f"{ms},{cid >> 12},0x{cid & 0x7FF:03X}," + ",".join(str(b) for b in data))
