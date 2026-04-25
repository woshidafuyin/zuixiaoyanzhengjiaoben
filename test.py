# -*- coding: utf-8 -*-
"""
CAPL Download 最小复刻脚本（Driver阶段）

当前完成：
1. 0x600 周期唤醒/保活
2. 10 83 功能寻址
3. 3E 80 功能寻址，按 CAPL 时机手动发送
4. 31 01 02 03
5. 10 02
6. 27 11 / 27 12
7. 2E F184
8. Driver 34 / 36 / 37
9. Driver DD02，按 CAPL driverData() 逻辑解析 .rsa 文本为 512 字节

当前还不是完整 App 全流程：
未包含 31 FF00、App 34/36/37、App DD02、31 FF01、11 reset、10 81、14 清DTC。
"""

import can
import time
import struct
import threading
import subprocess
import re


# =========================
# CAN 配置
# =========================
BUS_KWARGS = {
    "interface": "vector",
    "channel": 0,
    "app_name": "CANoe",
}

BUS = None


FUN_ID = 0x7DF
PHY_ID = 0x71F
RES_ID = 0x79F
PAD = 0x55

# 0x600 周期唤醒/保活
KEEP_ALIVE_ID = 0x600
KEEP_ALIVE_DATA = b"\x00" * 8
KEEP_ALIVE_PERIOD = 0.10
PRE_WAKEUP_TIME = 1.5


# =========================
# 路径配置：按你本机改
# =========================
PY32 = r"E:\upPC\yanzhengjiaoben\venv32\Scripts\python.exe"
KEYGEN = r"E:\upPC\yanzhengjiaoben\keygen_worker_auto.py"
DLL = r"E:\upPC\devtools\shuaxiewenjian\CHERY_E0Y_UPDATE23231115.dll"

S19_DRIVER = r"E:\upPC\devtools\shuaxiewenjian\ARS1.33_702000139AA_S0000054429_FLD_020001.s19"
RSA_DRIVER = r"E:\upPC\devtools\shuaxiewenjian\CIR_S0000054429_020001_FLD1_MCU_UDS_20260417.rsa"


# =========================
# Driver 地址：来自 CANoe 面板
# =========================
DRIVER_START = 0x0049C038
DRIVER_LEN = 0x00001EB8


def hx(data: bytes) -> str:
    return " ".join(f"{x:02X}" for x in data)


class PeriodicSender:
    def __init__(self, bus, arb_id, payload, period_s, name=""):
        self.bus = bus
        self.arb_id = arb_id
        self.payload = payload
        self.period_s = period_s
        self.name = name or f"0x{arb_id:03X}"
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        print(f"[INFO] start periodic sender: {self.name}, period={self.period_s * 1000:.0f} ms")
        self._thread.start()

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        print(f"[INFO] stop periodic sender: {self.name}")

    def _worker(self):
        while not self._stop:
            try:
                msg = can.Message(
                    arbitration_id=self.arb_id,
                    data=self.payload,
                    is_extended_id=False,
                    is_fd=False,
                    bitrate_switch=False,
                )
                self.bus.send(msg)
            except Exception as e:
                print("[WARN] periodic sender error:", repr(e))
            time.sleep(self.period_s)


def send(arbid: int, data: bytes):
    if len(data) != 8:
        raise ValueError(f"CAN frame must be 8 bytes, got {len(data)}: {hx(data)}")

    msg = can.Message(
        arbitration_id=arbid,
        data=data,
        is_extended_id=False,
        is_fd=False,
        bitrate_switch=False,
    )
    BUS.send(msg)
    print(f"[TX] 0x{arbid:03X}: {hx(data)}")


def recv(timeout: float = 2.0):
    end = time.time() + timeout
    while time.time() < end:
        m = BUS.recv(0.05)
        if m is None:
            continue
        if m.arbitration_id != RES_ID:
            continue

        data = bytes(m.data)
        print(f"[RX] 0x{m.arbitration_id:03X}: {hx(data)}")
        return data

    return None


def drain_bus(duration: float = 0.3):
    end = time.time() + duration
    while time.time() < end:
        m = BUS.recv(0.01)
        if m is None:
            continue
        if m.arbitration_id == RES_ID:
            print(f"[DRAIN] 0x{m.arbitration_id:03X}: {hx(bytes(m.data))}")

def wait_sid(sid: int, timeout: float = 5.0):
    positive_sid = (sid + 0x40) & 0xFF
    end = time.time() + timeout

    while time.time() < end:
        d = recv(timeout=0.2)
        if d is None:
            continue

        if len(d) >= 2 and d[1] == positive_sid:
            return d

        if len(d) >= 3 and d[2] == positive_sid:
            return d

        if len(d) >= 4 and d[1] == 0x7F:
            if d[2] != sid:
                print(f"[SKIP] unrelated NRC for SID 0x{d[2]:02X}: {hx(d)}")
                continue

            nrc = d[3]
            if nrc == 0x78:
                print(f"[WAIT] NRC 78 pending for SID 0x{sid:02X}")
                continue

            raise RuntimeError(f"NRC for SID 0x{sid:02X}: NRC=0x{nrc:02X}, raw={hx(d)}")

        print(f"[SKIP] waiting 0x{positive_sid:02X}, got {hx(d)}")

    raise TimeoutError(f"Timeout waiting positive response for SID 0x{sid:02X}")
# =========================
# ISO-TP 发送
# =========================
def send_uds(payload: bytes, functional: bool = False, pad: int = PAD):
    tx_id = FUN_ID if functional else PHY_ID

    # ===== 单帧 =====
    if len(payload) <= 7:
        frame = bytes([len(payload)]) + payload
        frame += bytes([pad]) * (8 - len(frame))
        send(tx_id, frame)
        return

    # ===== 多帧 =====
    length = len(payload)
    if length > 0xFFF:
        raise ValueError(f"payload too large for classic ISO-TP: {length}")

    # ---------- First Frame ----------
    ff = bytes([
        0x10 | ((length >> 8) & 0x0F),
        length & 0xFF
    ]) + payload[:6]

    ff += bytes([pad]) * (8 - len(ff))
    send(tx_id, ff)

    # ---------- 等 FlowControl（🔥关键修复） ----------
    fc = None
    t_end = time.time() + 2.0

    while time.time() < t_end:
        msg = recv(timeout=0.1)
        if msg is None:
            continue

        print("[RX for FC]", hx(msg))   # 🔥调试用

        # 只认 FC（0x30）
        if msg[0] == 0x30:
            fc = msg
            break

    if fc is None:
        raise TimeoutError("No FlowControl after FF")

    print("[FC OK]", hx(fc))

    # ---------- Consecutive Frames ----------
    pos = 6
    sn = 1

    while pos < length:
        chunk = payload[pos:pos + 7]

        frame = bytes([0x20 | (sn & 0x0F)]) + chunk
        frame += bytes([pad]) * (8 - len(frame))

        send(tx_id, frame)

        pos += len(chunk)
        sn = (sn + 1) & 0x0F
def recv_uds_payload(timeout: float = 5.0):
    first = recv(timeout=timeout)
    if first is None:
        raise TimeoutError("UDS response timeout")

    pci_type = first[0] >> 4

    if pci_type == 0x0:
        sf_len = first[0] & 0x0F
        return first[1:1 + sf_len]

    if pci_type == 0x1:
        total_len = ((first[0] & 0x0F) << 8) | first[1]
        buf = bytearray(first[2:])

        print(f"[ISO-TP RX] FF total_len={total_len}, already={len(buf)}")

        fc = b"\x30\x00\x00\x00\x00\x00\x00\x00"
        print("[FC] send:", hx(fc))
        send(PHY_ID, fc)

        time.sleep(0.02)

        end = time.time() + timeout

        while len(buf) < total_len:
            if time.time() > end:
                print(f"[WARN] CF timeout, got {len(buf)}/{total_len}, use partial data")
                return bytes(buf)

            cf = recv(timeout=0.1)

            if cf is None:
                continue

            if cf[0] == 0x30:
                print("[INFO] ignore FC-like frame:", hx(cf))
                continue

            if (cf[0] >> 4) != 0x2:
                print("[SKIP] expected CF, got:", hx(cf))
                continue

            buf.extend(cf[1:])

        return bytes(buf[:total_len])

    raise RuntimeError(f"Unsupported PCI frame: {hx(first)}")
# =========================
# 文件解析
# =========================
def s19_to_bin(path: str, start: int, length: int) -> bytes:
    mem = {}

    with open(path, "r", encoding="ascii", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("S3"):
                continue

            count = int(line[2:4], 16)
            addr = int(line[4:12], 16)
            data_hex = line[12:12 + (count - 5) * 2]
            raw = bytes.fromhex(data_hex)

            for i, b in enumerate(raw):
                mem[addr + i] = b

    if not mem:
        raise RuntimeError(f"No S3 records found: {path}")

    return bytes(mem.get(a, 0xFF) for a in range(start, start + length))

def parse_rsa_text(path: str) -> bytes:
    import re

    with open(path, "r", encoding="ascii", errors="ignore") as f:
        text = f.read()

    # 支持 0x / 0X
    tokens = re.findall(r"0[xX]([0-9a-fA-F]{1,2})", text)

    if len(tokens) < 512:
        raise RuntimeError(f"RSA parsed too short: {len(tokens)} bytes, need 512")

    data = bytes(int(x, 16) & 0xFF for x in tokens[:512])

    # 🔥 强校验（关键）
    print("sig len =", len(data))
    print("sig head =", " ".join(f"{b:02X}" for b in data[:16]))

    return data

def keygen(seed: bytes) -> bytes:
    try:
        out = subprocess.check_output(
            [PY32, KEYGEN, DLL, seed.hex()],
            stderr=subprocess.STDOUT
        ).decode(errors="ignore")
    except subprocess.CalledProcessError as e:
        out = e.output.decode(errors="ignore")
        print("[KEYGEN RAW - FAILED]")
        print(out)
        raise

    print("[KEYGEN RAW]")
    print(out)

    m = re.search(r"KEY_HEX\s*=\s*([0-9a-fA-F]+)", out)
    if not m:
        raise RuntimeError("KEY_HEX not found in keygen output")

    key = bytes.fromhex(m.group(1))
    if not key:
        raise RuntimeError("KEY_HEX is empty")

    return key
def run():
    global BUS

    BUS = can.Bus(**BUS_KWARGS)
    print("[OK] bus opened")

    ka = PeriodicSender(
        bus=BUS,
        arb_id=KEEP_ALIVE_ID,
        payload=KEEP_ALIVE_DATA,
        period_s=KEEP_ALIVE_PERIOD,
        name="keep_alive_0x600",
    )

    try:
        print("==== START CAPL DRIVER DOWNLOAD MIN ====")
        print("BUS_KWARGS =", BUS_KWARGS)

        time.sleep(0.2)
        drain_bus(0.5)

        # ===== 1. 10 83 =====
        send(FUN_ID, b"\x02\x10\x83\x55\x55\x55\x55\x55")

        # ===== 2. 3E 80 =====
        send(FUN_ID, b"\x02\x3E\x80\x55\x55\x55\x55\x55")
        time.sleep(0.1)

        # ===== 3. 31 0203 =====
        send(PHY_ID, b"\x04\x31\x01\x02\x03\x55\x55\x55")
        resp31 = wait_sid(0x31, timeout=2.0)
        print("[OK] 31 0203 =", hx(resp31))
        time.sleep(0.1)

        # ===== 4. 10 02 =====
        send(PHY_ID, b"\x02\x10\x02\x55\x55\x55\x55\x55")
        resp10 = wait_sid(0x10, timeout=6.0)
        print("[OK] 10 02 =", hx(resp10))
        time.sleep(2.2)

        # ===== 5. 27 11 =====
        send(PHY_ID, b"\x02\x27\x11\xFF\xFF\xFF\xFF\xFF")

        seed_resp = recv_uds_payload(timeout=5.0)
        if seed_resp is None or len(seed_resp) < 6:
            print("[WARN] incomplete seed response:", seed_resp)
            return

        print("[UDS] 27 11 payload =", hx(seed_resp))

        if seed_resp[0] != 0x67 or seed_resp[1] != 0x11:
            raise RuntimeError(f"Unexpected 27 seed response: {hx(seed_resp)}")

        seed = seed_resp[2:]
        if len(seed) > 16:
            seed = seed[:16]

        print("[SEED]", hx(seed), "len=", len(seed))

        # ===== 6. KEYGEN =====
        key = keygen(seed)
        print("[KEY]", hx(key), "len=", len(key))

        # ===== 7. 27 12 =====
        send_uds(b"\x27\x12" + key)

        resp27 = wait_sid(0x27, timeout=2.0)
        print("[OK] 27 12 =", hx(resp27))
        time.sleep(0.1)

        # ===== 8. 2E F184 =====
        finger = b"\x26\x04\x23" + b"\xFF" * 16
        send_uds(b"\x2E\xF1\x84" + finger)
        resp2e = wait_sid(0x2E, timeout=2.0)
        print("[OK] 2E F184 =", hx(resp2e))
        time.sleep(0.1)

        # ===== 9. Driver =====
        driver_data = s19_to_bin(S19_DRIVER, DRIVER_START, DRIVER_LEN)
        print(f"[DRIVER] start=0x{DRIVER_START:08X} len=0x{DRIVER_LEN:08X}")

        # ===== 10. 34 =====
        req34 = struct.pack(">BBBII", 0x34, 0x00, 0x44, DRIVER_START, DRIVER_LEN)
        send_uds(req34)
        resp34 = wait_sid(0x34, timeout=2.0)
        print("[OK] 34 DRIVER =", hx(resp34))

        block_length = (resp34[5] << 8) | resp34[6]
        chunk_size = block_length - 2
        print("chunk_size =", chunk_size)

        # ===== 11. 36 =====
        total_blocks = (len(driver_data) + chunk_size - 1) // chunk_size

        pos = 0
        seq = 1

        print(">>> WAITING 36 FINISH <<<")

        while pos < len(driver_data):
            print(f"[36] block={seq}/{total_blocks}")

            chunk = driver_data[pos:pos + chunk_size]
            send_uds(bytes([0x36, seq & 0xFF]) + chunk)
            wait_sid(0x36, timeout=3.0)
            time.sleep(0.05)

            pos += len(chunk)
            seq = (seq + 1) & 0xFF

        print("36 DONE")

        # ===== 12. 37 =====
        # ===== 12. 37 =====
        send_uds(b"\x37")
        resp37 = wait_sid(0x37, timeout=2.0)
        print("[OK] 37 =", hx(resp37))

        # CAPL: server_37 后 TxMsgSrever 内部 50ms
        time.sleep(0.05)

        # CAPL: 37 后周期 3E80
        send(FUN_ID, b"\x02\x3E\x80\x55\x55\x55\x55\x55")

        # CAPL: driverData() 后 testWaitForTimeout(1000)
        time.sleep(1.0)

        # ===== 13. 31 DD02 =====
        sig = parse_rsa_text(RSA_DRIVER)

        print("sig len =", len(sig))
        print("sig head =", hx(sig[:16]))
        print("sig tail =", hx(sig[-16:]))

        assert len(sig) == 512, f"RSA len error: {len(sig)}"
        payload = (
                b"\x31\x01\xDD\x02"
                + sig
        )

        print("payload len =", len(payload))

        send_uds(payload)

        resp_dd02 = wait_sid(0x31, timeout=5.0)
        print("[OK] 31 DD02 =", hx(resp_dd02))

        print("==== DRIVER DONE ====")

    finally:
        try:
            ka.stop()
        except:
            pass

        if BUS is not None:
            BUS.shutdown()
            print("[OK] bus closed")

if __name__ == "__main__":
    run()