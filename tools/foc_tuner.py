#!/usr/bin/env python3
"""FOC PID 自动调参 — Windows
依据固件 comm_host.c 协议：
  mode <idle|pos|spd|cur>  → 切换控制模式
  enable / disable         → 电机使能
  target <value>           → 按当前模式设目标
  pid <loop> <kp> <ki>     → 设PID (loop: 0=pos 1=spd 2=id 3=iq)
  monitor [var ...]        → 开/关遥测
  status                   → 状态行

用法: python foc_tuner.py COM3 [baud]
      python foc_tuner.py COM3 speed
      python foc_tuner.py COM3 pos
      python foc_tuner.py COM3 cur
依赖: pip install pyserial numpy
"""
import sys, time, re, threading, queue
import serial
import numpy as np

# ─── 配置 ───
BAUD          = 115200
SETTLE_S      = 0.5
RECORD_S      = 2.0
MAX_ITERS     = 12
OVERSHOOT_TGT = 0.08
RISE_TGT_SPD  = 0.15   # 速度环目标上升时间 (s)
RISE_TGT_POS  = 0.30   # 位置环
RISE_TGT_CUR  = 0.02   # 电流环 (monitor 10ms采样，2个点=20ms)

# ─── 串口 ───
class FocLink:
    def __init__(self, port, baud=BAUD):
        # dsrdtr=False / rtscts=False: 防止 pyserial 开端口时切换 DTR/RTS
        # 导致 USB 转串口适配器拉低 MCU RESET → 复位 → 对齐 ~2.5s → 通信超时
        self.ser = serial.Serial(port, baud, timeout=0.01,
                                 dsrdtr=False, rtscts=False, xonxoff=False)
        self.ser.dtr = False
        self.ser.rts = False
        self.rxq = queue.Queue()
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while not self._stop:
            try:
                d = self.ser.read(4096)
            except serial.SerialException:
                break
            if d:
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode("ascii", errors="ignore").strip()
                    if s:
                        self.rxq.put(s)

    def send(self, cmd):
        self.ser.write((cmd + "\r\n").encode())
        time.sleep(0.05)

    def flush(self):
        while not self.rxq.empty():
            try: self.rxq.get_nowait()
            except queue.Empty: break

    def wait_ack(self, timeout=1.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                line = self.rxq.get(timeout=0.05)
            except queue.Empty:
                continue
            if line.startswith("OK") or line.startswith("ERR"):
                return line
        return None

    def query_status(self, retries=3):
        """发 status 命令，解析返回的状态字典。
        MCU 命令处理有 1-2 帧延迟，单次可能收不到，自动重试。"""
        for attempt in range(retries):
            self.flush()
            self.send("status")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                try:
                    line = self.rxq.get(timeout=0.05)
                except queue.Empty:
                    continue
                if "STATUS:" in line:
                    info = {}
                    for m in re.finditer(r"(\w+)=([\w.\-]+)", line):
                        info[m.group(1)] = m.group(2)
                    return info
            time.sleep(0.3)
        return None

    def read_monitor(self, duration_s):
        """采集 monitor 数据"""
        self.flush()
        data = {"t": [], "pos": [], "spd": [], "iq": [], "id": [], "vbus": []}
        t0 = time.monotonic()
        pat = re.compile(r"t=([\d.eE+-]+),?\s+(.*)")
        kv  = re.compile(r"(\w+)=([\d.eE+-]+)")
        while time.monotonic() - t0 < duration_s:
            try:
                line = self.rxq.get(timeout=0.05)
            except queue.Empty:
                continue
            m = pat.match(line)
            if not m:
                continue
            try:
                data["t"].append(float(m.group(1)))
            except ValueError:
                continue
            for k, v in kv.findall(m.group(2)):
                if k in data:
                    try:
                        data[k].append(float(v))
                    except ValueError:
                        data[k].append(0.0)
        n = len(data["t"])
        for k in data:
            if k != "t" and len(data[k]) < n:
                data[k] += [0.0] * (n - len(data[k]))
        return data if n > 10 else None

    def close(self):
        self._stop = True
        self.ser.close()

# ─── 模式管理 ───
def enter_mode(link, mode_name):
    """安全切换模式：disable → idle → target mode → 验证"""
    print(f"  切换模式 → {mode_name}")
    link.send("disable")
    link.wait_ack()
    time.sleep(0.1)

    # 先回 idle 清状态
    link.send("mode idle")
    link.wait_ack()
    time.sleep(0.1)

    # 切目标模式
    ack = link.send(f"mode {mode_name}") or ""
    r = link.wait_ack()
    if r and "ERR" in r:
        print(f"  !! 模式切换失败: {r}")
        return False
    time.sleep(0.1)

    # 验证
    st = link.query_status()
    if not st:
        print("  !! 无法读取状态")
        return False

    cur_mode = st.get("mode", "?")
    fault = int(st.get("fault", "1"))
    state = st.get("state", "?")

    mode_map = {"idle": "IDLE", "spd": "SPEED", "pos": "POSITION", "cur": "TORQUE"}
    expected = mode_map.get(mode_name, mode_name.upper())

    if cur_mode != expected:
        print(f"  !! 模式不匹配: 期望={expected} 实际={cur_mode}")
        return False
    if fault != 0:
        print(f"  !! 存在故障 fault={fault}，请先排除")
        return False

    print(f"  ✓ 模式={cur_mode} 状态={state} 故障=0 vbus={st.get('vbus','?')}V")
    return True

def safe_enable(link):
    """使能电机，检查故障"""
    st = link.query_status()
    if st and int(st.get("fault", "1")) != 0:
        print(f"  !! 故障未清除 (fault={st['fault']})，跳过使能")
        return False
    link.send("enable")
    r = link.wait_ack()
    if r and "ERR" in r:
        print(f"  !! 使能失败: {r}")
        return False
    return True

# ─── 数据合并 ───
def _merge(a, b):
    if a is None: return b
    if b is None: return a
    out = {}
    for k in a:
        out[k] = a[k] + b[k]
    return out

# ─── 阶跃分析 ───
def analyze_step(fb, target_val, dt, rise_target, baseline_len=0):
    fb = np.array(fb, dtype=float)
    if len(fb) < 20:
        return 999, 999, 999, 999

    bl = baseline_len if baseline_len > 0 else 5
    y0 = np.mean(fb[:bl])
    resp = fb - y0
    final = target_val - y0
    if abs(final) < 1e-6:
        return 0, 999, 0, 0

    peak = np.max(np.abs(resp))
    overshoot = max(0, (peak - abs(final)) / abs(final))

    t10 = t90 = None
    for i, v in enumerate(resp):
        if t10 is None and abs(v) >= abs(final) * 0.1:
            t10 = i
        if t10 is not None and t90 is None and abs(v) >= abs(final) * 0.9:
            t90 = i
            break
    rise_time = (t90 - t10) * dt if (t10 is not None and t90 is not None) else 999.0

    tail = resp[int(len(resp)*0.8):]
    sse = abs(np.mean(tail) - final) / abs(final) if len(tail) > 0 else 0

    sc = np.sum(np.abs(np.diff(np.sign(resp - np.mean(tail)))) > 0)
    osc = sc / max(len(resp) * dt, 0.1)

    return overshoot, rise_time, sse, osc

# ─── 速度环调参 (loop=1) ───
def tune_speed(link):
    print("\n" + "="*50)
    print(" 速度环自动调参 (loop=1)")
    print("="*50)

    if not enter_mode(link, "spd"):
        return None

    link.send("monitor spd iq")
    link.wait_ack()
    time.sleep(0.2)

    kp, ki = 0.18, 0.15
    best = None

    for it in range(MAX_ITERS):
        print(f"\n[迭代 {it+1}/{MAX_ITERS}] Kp={kp:.4f} Ki={ki:.4f}")
        link.send(f"pid 1 {kp:.4f} {ki:.4f}")
        link.wait_ack()
        time.sleep(0.2)

        # 正向阶跃 0→60
        link.send("target 0")
        link.wait_ack()
        time.sleep(SETTLE_S)
        if not safe_enable(link):
            break
        time.sleep(0.3)
        link.flush()
        base1 = link.read_monitor(0.3)
        link.send("target 60")
        link.wait_ack()
        resp1 = link.read_monitor(RECORD_S)
        d1 = _merge(base1, resp1)

        # 反向 60→-60
        time.sleep(0.3)
        link.flush()
        base2 = link.read_monitor(0.3)
        link.send("target -60")
        link.wait_ack()
        resp2 = link.read_monitor(RECORD_S)
        d2 = _merge(base2, resp2)

        link.send("disable")
        link.wait_ack()

        if not d1 or not d2:
            print("  !! 无数据，检查 monitor / 串口")
            break

        dt = np.mean(np.diff(d1["t"])) if len(d1["t"]) > 1 else 0.005
        bl1 = len(base1["t"]) if base1 else 5
        bl2 = len(base2["t"]) if base2 else 5
        ov1, rt1, sse1, osc1 = analyze_step(d1["spd"], 60.0, dt, RISE_TGT_SPD, bl1)
        ov2, rt2, sse2, osc2 = analyze_step(d2["spd"], -120.0, dt, RISE_TGT_SPD, bl2)

        ov  = max(ov1, ov2)
        rt  = max(rt1, rt2)
        sse = max(sse1, sse2)
        osc = max(osc1, osc2)

        print(f"  超调={ov*100:.1f}%  上升={rt*1000:.0f}ms  "
              f"稳态误差={sse*100:.1f}%  振荡={osc:.1f}/s")

        score = ov*3 + min(rt/RISE_TGT_SPD, 5) + sse*2 + osc*0.3
        if best is None or score < best[0]:
            best = (score, kp, ki, ov, rt, sse, osc)

        if ov > OVERSHOOT_TGT * 2:
            kp *= 0.65; ki *= 0.55
        elif ov > OVERSHOOT_TGT:
            kp *= 0.85; ki *= 0.75
        elif rt > RISE_TGT_SPD * 1.5:
            kp *= 1.35; ki *= 1.1
        elif sse > 0.05:
            ki *= 1.5
        elif osc > 4.0:
            ki *= 0.6
        else:
            print("  ✓ 收敛")
            break

        kp = max(0.005, min(kp, 2.0))
        ki = max(0.001, min(ki, 5.0))

    link.send("monitor")
    if best:
        print(f"\n★ 速度环最优: Kp={best[1]:.4f} Ki={best[2]:.4f}")
        print(f"  超调={best[3]*100:.1f}% 上升={best[4]*1000:.0f}ms "
              f"稳态={best[5]*100:.1f}% 振荡={best[6]:.1f}/s")
        link.send(f"pid 1 {best[1]:.4f} {best[2]:.4f}")
        link.wait_ack()
    return best

# ─── 位置环调参 (loop=0) ───
def tune_position(link):
    print("\n" + "="*50)
    print(" 位置环自动调参 (loop=0)")
    print("="*50)

    if not enter_mode(link, "pos"):
        return None

    link.send("monitor pos spd")
    link.wait_ack()
    time.sleep(0.2)

    kp = 8.0
    best = None

    for it in range(MAX_ITERS):
        print(f"\n[迭代 {it+1}/{MAX_ITERS}] Kp_pos={kp:.3f}")
        link.send(f"pid 0 {kp:.4f} 0")
        link.wait_ack()
        time.sleep(0.2)

        link.send("target 0")
        link.wait_ack()
        time.sleep(SETTLE_S)
        if not safe_enable(link):
            break
        time.sleep(0.3)
        link.flush()
        base_p = link.read_monitor(0.3)
        link.send("target 3.0")
        link.wait_ack()
        resp_p = link.read_monitor(RECORD_S + 0.5)
        d = _merge(base_p, resp_p)
        link.send("disable")
        link.wait_ack()

        if not d:
            print("  !! 无数据")
            break

        dt = np.mean(np.diff(d["t"])) if len(d["t"]) > 1 else 0.005
        bl_p = len(base_p["t"]) if base_p else 5
        ov, rt, sse, osc = analyze_step(d["pos"], 3.0, dt, RISE_TGT_POS, bl_p)
        print(f"  超调={ov*100:.1f}%  上升={rt*1000:.0f}ms  "
              f"稳态误差={sse*100:.1f}%  振荡={osc:.1f}/s")

        score = ov*3 + min(rt/RISE_TGT_POS, 5) + sse*2 + osc*0.3
        if best is None or score < best[0]:
            best = (score, kp, ov, rt, sse, osc)

        if ov > 0.12:
            kp *= 0.65
        elif ov > 0.06:
            kp *= 0.85
        elif rt > RISE_TGT_POS * 1.5:
            kp *= 1.3
        elif sse > 0.03:
            kp *= 1.15
        elif osc > 3.0:
            kp *= 0.75
        else:
            print("  ✓ 收敛")
            break

        kp = max(0.3, min(kp, 15.0))

    link.send("monitor")
    if best:
        print(f"\n★ 位置环最优: Kp={best[1]:.4f}")
        print(f"  超调={best[2]*100:.1f}% 上升={best[3]*1000:.0f}ms "
              f"稳态={best[4]*100:.1f}% 振荡={best[5]:.1f}/s")
        link.send(f"pid 0 {best[1]:.4f} 0")
        link.wait_ack()
    return best

# ─── 电流环调参 (loop=2,3) ───
def tune_current(link):
    print("\n" + "="*50)
    print(" 电流环自动调参 (loop=2 id / loop=3 iq)")
    print("="*50)

    if not enter_mode(link, "cur"):
        return None

    link.send("monitor iq id")
    link.wait_ack()
    time.sleep(0.2)

    # 先调 iq 环 (loop=3)，再调 id 环 (loop=2)
    results = {}
    for loop_id, var_name, step_val in [(3, "iq", 1.0), (2, "id", 0.5)]:
        print(f"\n--- {var_name} 环 (loop={loop_id}) 阶跃 {step_val}A ---")
        # 25kHz电流环: 固件默认 Kp=0.5 Ki=10，从这里开始搜
        kp, ki = 0.5, 10.0
        best = None

        for it in range(MAX_ITERS):
            print(f"\n[{var_name} 迭代 {it+1}/{MAX_ITERS}] Kp={kp:.4f} Ki={ki:.4f}")
            link.send(f"pid {loop_id} {kp:.4f} {ki:.4f}")
            link.wait_ack()
            time.sleep(0.2)

            link.send("target 0")
            link.wait_ack()
            time.sleep(SETTLE_S)
            if not safe_enable(link):
                break
            time.sleep(0.2)
            link.flush()
            base_c = link.read_monitor(0.2)
            link.send(f"target {step_val}")
            link.wait_ack()
            resp_c = link.read_monitor(1.0)  # 电流环快，1s够
            d = _merge(base_c, resp_c)
            link.send("disable")
            link.wait_ack()

            if not d:
                print("  !! 无数据")
                break

            dt = np.mean(np.diff(d["t"])) if len(d["t"]) > 1 else 0.00004
            bl_c = len(base_c["t"]) if base_c else 5
            ov, rt, sse, osc = analyze_step(d[var_name], step_val, dt, RISE_TGT_CUR, bl_c)
            print(f"  超调={ov*100:.1f}%  上升={rt*1000:.1f}ms  "
                  f"稳态误差={sse*100:.1f}%  振荡={osc:.1f}/s")

            score = ov*3 + min(rt/RISE_TGT_CUR, 5) + sse*2 + osc*0.1
            if best is None or score < best[0]:
                best = (score, kp, ki, ov, rt, sse, osc)

            if ov > 0.15:
                kp *= 0.7; ki *= 0.6
            elif ov > 0.05:
                kp *= 0.85; ki *= 0.8
            elif rt > RISE_TGT_CUR * 2:
                kp *= 1.4; ki *= 1.2
            elif sse > 0.03:
                ki *= 1.5
            elif osc > 10.0:
                ki *= 0.5
            else:
                print("  ✓ 收敛")
                break

            kp = max(0.1, min(kp, 20.0))
            ki = max(1.0, min(ki, 5000.0))

        if best:
            print(f"\n★ {var_name} 环最优: Kp={best[1]:.4f} Ki={best[2]:.4f}")
            link.send(f"pid {loop_id} {best[1]:.4f} {best[2]:.4f}")
            link.wait_ack()
            results[var_name] = best

    link.send("monitor")
    return results

# ─── 主 ───
def main():
    if len(sys.argv) < 2:
        print("用法: python foc_tuner.py COM3 [baud]")
        print("      python foc_tuner.py COM3 speed")
        print("      python foc_tuner.py COM3 pos")
        print("      python foc_tuner.py COM3 cur")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else BAUD
    mode = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].isdigit() else "all"

    print(f"连接 {port} @{baud} ...")
    link = FocLink(port, baud)
    time.sleep(0.3)

    # 排空上电 banner + ring buffer 残留乱码
    # 先发几个空行把可能的半截命令冲掉，再清空接收队列
    for _ in range(3):
        link.send("")
    time.sleep(0.2)
    link.flush()

    # 验证通信 — MCU 命令处理有 1-2 帧延迟，query_status 内部已带重试
    st = link.query_status()

    if not st:
        print("  !! 未收到 STATUS，检查接线/波特率/MCU 是否复位")
        link.close()
        sys.exit(1)

    print(f"  MCU: mode={st.get('mode')} state={st.get('state')} "
          f"fault={st.get('fault')} vbus={st.get('vbus')}V")

    if int(st.get("fault", "1")) != 0:
        print("  !! MCU 存在故障，请先排除后再调参")
        link.close()
        sys.exit(1)

    try:
        if mode in ("all", "cur", "current"):
            tune_current(link)
        if mode in ("all", "speed", "spd"):
            tune_speed(link)
        if mode in ("all", "pos", "position"):
            tune_position(link)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        link.send("disable")
        link.send("mode idle")
        link.send("monitor")
        time.sleep(0.1)
        link.close()
        print("\n电机已 disable，串口已断开")

if __name__ == "__main__":
    main()