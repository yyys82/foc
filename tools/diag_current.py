"""诊断: 电流环阶跃 — 抓 id/iq 原始值，判断采样/角度问题"""
import sys, os, time, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import serial, threading, queue

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = 115200

class Link:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.01,
                                 dsrdtr=False, rtscts=False, xonxoff=False)
        self.ser.dtr = False; self.ser.rts = False
        self.rxq = queue.Queue()
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while not self._stop:
            try: d = self.ser.read(4096)
            except: break
            if d:
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("ascii", errors="ignore").strip()
                    if s: self.rxq.put(s)

    def send(self, cmd):
        self.ser.write((cmd + "\r\n").encode()); time.sleep(0.05)

    def flush(self):
        while not self.rxq.empty():
            try: self.rxq.get_nowait()
            except: break

    def wait_ack(self, timeout=1.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try: line = self.rxq.get(timeout=0.05)
            except queue.Empty: continue
            if line.startswith("OK") or line.startswith("ERR"): return line
        return None

    def collect(self, duration_s):
        lines = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            try: lines.append(self.rxq.get(timeout=0.05))
            except queue.Empty: pass
        return lines

    def close(self): self._stop = True

def parse_monitor(lines):
    """解析 t=xxx, id=xxx iq=xxx 格式"""
    data = {"t": [], "id": [], "iq": []}
    pat = re.compile(r"t=([\d.eE+-]+),?\s+(.*)")
    kv = re.compile(r"(\w+)=([\d.eE+-]+)")
    for line in lines:
        m = pat.match(line)
        if not m: continue
        try: data["t"].append(float(m.group(1)))
        except: continue
        for k, v in kv.findall(m.group(2)):
            if k in data:
                try: data[k].append(float(v))
                except: data[k].append(0.0)
    return data

link = Link(PORT, BAUD)
time.sleep(0.3)

# 排空残留
for _ in range(3): link.send("")
time.sleep(0.2)
link.flush()

print(f"=== 诊断: {PORT} ===\n")

# 1. 状态
link.send("status")
time.sleep(0.3)
lines = link.collect(0.5)
for l in lines:
    if "STATUS" in l: print(f"[状态] {l}")

# 2. 切到电流模式
link.send("mode cur")
print(f"[mode] {link.wait_ack()}")

# 3. 监控 id iq
link.send("monitor id iq")
print(f"[monitor] {link.wait_ack()}")
time.sleep(0.2)

# 4. 使能前 — 读零漂
link.send("target 0")
link.wait_ack()
time.sleep(0.3)
link.flush()
base_lines = link.collect(1.0)
base = parse_monitor(base_lines)
if base["iq"]:
    import statistics
    print(f"\n[使能前零漂] id={statistics.mean(base['id']):.4f}A  iq={statistics.mean(base['iq']):.4f}A  (n={len(base['iq'])})")
else:
    print("\n[使能前] 无数据!")

# 5. 使能 + 阶跃 0→1A
link.send("enable")
print(f"[enable] {link.wait_ack()}")
time.sleep(0.3)

link.send("target 0")
link.wait_ack()
time.sleep(0.5)
link.flush()
zero_lines = link.collect(0.5)
zero = parse_monitor(zero_lines)

link.send("target 1.0")
print(f"[target] {link.wait_ack()}")
time.sleep(0.1)
link.flush()
step_lines = link.collect(2.0)
step = parse_monitor(step_lines)

link.send("disable")
link.wait_ack()

if step["iq"]:
    import statistics
    n = len(step["iq"])
    tail_start = int(n * 0.7)
    iq_tail = step["iq"][tail_start:]
    id_tail = step["id"][tail_start:]
    print(f"\n[阶跃 0→1A] 共 {n} 个采样点")
    print(f"  iq: min={min(step['iq']):.4f} max={max(step['iq']):.4f} tail_mean={statistics.mean(iq_tail):.4f}")
    print(f"  id: min={min(step['id']):.4f} max={max(step['id']):.4f} tail_mean={statistics.mean(id_tail):.4f}")
    print(f"  SSE = {abs(1.0 - statistics.mean(iq_tail))/1.0*100:.1f}%")
    print(f"  |id/iq| = {abs(statistics.mean(id_tail)/max(abs(statistics.mean(iq_tail)),0.001)):.3f}  (正常应≈0)")
    
    # 打印前20个和后20个原始值
    print(f"\n  前20个 iq: {[f'{v:.3f}' for v in step['iq'][:20]]}")
    print(f"  后20个 iq: {[f'{v:.3f}' for v in step['iq'][-20:]]}")
    print(f"  前20个 id: {[f'{v:.3f}' for v in step['id'][:20]]}")
else:
    print("\n[阶跃] 无数据!")

# 6. 关 monitor
link.send("monitor")
link.wait_ack()
link.close()
print("\n=== 诊断完成 ===")
