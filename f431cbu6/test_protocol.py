#!/usr/bin/env python3
"""
test_protocol.py — 验证 moteus multiplex 协议编解码
无需硬件，纯软件测试帧格式是否正确。
"""

import struct
import sys
import io
sys.path.insert(0, r'C:\Users\15034\Desktop\w\foc\keil_project\moteus_src\lib\python')

from moteus import multiplex as mp
from moteus import protocol
from moteus.protocol import Register, Writer


def test_position_command():
    """模拟发送 position mode 命令帧"""
    print("=== Test: Position Command Frame ===")

    buf = io.BytesIO()
    writer = Writer(buf)

    # Write MODE = 10 (POSITION) as INT8
    combiner = mp.WriteCombiner(writer, 0x00, Register.MODE, [mp.INT8])
    if combiner.maybe_write():
        writer.write_int(10, mp.INT8)

    # Write COMMAND_POSITION = 0.5 rev as F32
    combiner = mp.WriteCombiner(writer, 0x00, Register.COMMAND_POSITION, [mp.F32])
    if combiner.maybe_write():
        writer.write_position(0.5, mp.F32)

    # Write COMMAND_VELOCITY = 0.0 as F32
    combiner = mp.WriteCombiner(writer, 0x00, Register.COMMAND_VELOCITY, [mp.F32])
    if combiner.maybe_write():
        writer.write_velocity(0.0, mp.F32)

    # Write COMMAND_FEEDFORWARD_TORQUE = 0.0 as F32
    combiner = mp.WriteCombiner(writer, 0x00, Register.COMMAND_FEEDFORWARD_TORQUE, [mp.F32])
    if combiner.maybe_write():
        writer.write_torque(0.0, mp.F32)

    frame = buf.getvalue()
    print(f"  Frame hex: {frame.hex()}")
    print(f"  Frame len: {len(frame)} bytes")

    # Parse it back
    result = protocol.parse_registers(frame)
    print(f"  Commands: {result.command}")
    assert Register.MODE in result.command
    assert result.command[Register.MODE] == 10
    assert abs(result.command[Register.COMMAND_POSITION] - 0.5) < 0.001
    print("  PASS ✓")


def test_query_frame():
    """模拟 query 帧 (读 mode/position/velocity/torque)"""
    print("\n=== Test: Query Frame ===")

    buf = io.BytesIO()
    writer = Writer(buf)

    # Read registers 0x000-0x003
    combiner = mp.WriteCombiner(writer, 0x10, Register.MODE,
                                [mp.INT8, mp.F32, mp.F32, mp.F32])
    combiner.maybe_write()
    combiner.maybe_write()
    combiner.maybe_write()
    combiner.maybe_write()

    frame = buf.getvalue()
    print(f"  Frame hex: {frame.hex()}")
    print(f"  Frame len: {len(frame)} bytes")

    # Parse
    result = protocol.parse_registers(frame)
    print(f"  Query regs: {result.query}")
    assert len(result.query) == 4
    print("  PASS ✓")


def test_reply_parse():
    """模拟设备回复帧并解析"""
    print("\n=== Test: Reply Frame Parse ===")

    buf = io.BytesIO()
    writer = Writer(buf)

    # Reply: MODE=10(INT8), POSITION=0.123(F32), VELOCITY=-0.5(F32), TORQUE=1.2(F32)
    combiner = mp.WriteCombiner(writer, 0x20, Register.MODE,
                                [mp.INT8, mp.F32, mp.F32, mp.F32])
    if combiner.maybe_write():
        writer.write_int(10, mp.INT8)
    if combiner.maybe_write():
        writer.write_position(0.123, mp.F32)
    if combiner.maybe_write():
        writer.write_velocity(-0.5, mp.F32)
    if combiner.maybe_write():
        writer.write_torque(1.2, mp.F32)

    frame = buf.getvalue()
    print(f"  Frame hex: {frame.hex()}")

    result = protocol.parse_registers(frame)
    print(f"  Response: {result.response}")
    assert result.response[Register.MODE] == 10
    assert abs(result.response[Register.POSITION] - 0.123) < 0.0001
    assert abs(result.response[Register.VELOCITY] - (-0.5)) < 0.0001
    assert abs(result.response[Register.TORQUE] - 1.2) < 0.001
    print("  PASS ✓")


def test_can_id():
    """验证 CAN ID 计算"""
    print("\n=== Test: CAN ID ===")
    device_id = 1
    source = 0
    # moteus: CAN ID = (source << 8) | destination
    can_id = (source << 8) | device_id
    print(f"  Host→Device: src={source}, dst={device_id} → CAN ID=0x{can_id:03X}")
    assert can_id == 0x001

    # Reply: (device << 8) | host_source
    reply_id = (device_id << 8) | source
    print(f"  Device→Host: src={device_id}, dst={source} → CAN ID=0x{reply_id:03X}")
    assert reply_id == 0x100
    print("  PASS ✓")


def test_varuint():
    """验证 varuint 编解码"""
    print("\n=== Test: Varuint ===")

    # Test encoding various values
    for val in [0, 1, 127, 128, 255, 256, 16383, 16384]:
        buf = bytearray()
        v = val
        while v > 0x7F:
            buf.append((v & 0x7F) | 0x80)
            v >>= 7
        buf.append(v)

        # Decode
        result = 0
        shift = 0
        for b in buf:
            result |= (b & 0x7F) << shift
            shift += 7
            if not (b & 0x80):
                break
        assert result == val, f"varuint failed for {val}: got {result}"
    print("  All varuint values pass ✓")


if __name__ == '__main__':
    print("moteus multiplex protocol verification\n")
    test_varuint()
    test_position_command()
    test_query_frame()
    test_reply_parse()
    test_can_id()
    print("\n=== ALL TESTS PASSED ===")
