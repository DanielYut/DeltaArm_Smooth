"""
台達 DRV 手臂連續路徑 (不停頓) —— 透過控制器 RL (Lua) 的 MovL + PASS()

Modbus GO 指令一次只能走一個點，每點都會減速停下。這個模組把整條路徑寫進控制器，
由控制器上的 Lua 程式用 MovL + PASS() 連續走完。

需要：
  - 控制器上用 DRAStudio 裝一次 loader.lua (之後不用再開 DRAStudio)
  - pip install pymodbus      (3.6 ~ 3.11 都可以)

用法 (程式裡)：
    from delta_smooth import DeltaArm, SmoothPath, Pt

    arm = DeltaArm("192.168.1.1")
    x, y, z, rx, ry, rz = arm.pose()
    path = [Pt(x + 30, y, z, rx, ry, rz, speed=10),
            Pt(x + 30, y + 30, z, rx, ry, rz, speed=10),
            Pt(x, y, z, rx, ry, rz, speed=10)]
    SmoothPath(arm).run(path)

命令列測試：
    python delta_smooth.py diag              讀控制器狀態 (不動)
    python delta_smooth.py test              送 Lua + 自我測試 (不動)
    python delta_smooth.py square            在目前高度畫 30 mm 方形 (會動！)
    python delta_smooth.py circle            在目前高度畫半徑 20 mm 圓形 (會動！)
    加 --ip 192.168.1.1 --rl-id 1 --unit 2 指定連線
"""
import inspect
import os
import struct
import sys
import time
from collections import namedtuple

from pymodbus.client import ModbusTcpClient

HERE = os.path.dirname(os.path.abspath(__file__))
LUA_PATH = os.path.join(HERE, "path_runner.lua")

# ---------------- 控制器 Modbus 位址 (DCV Modbus Address Manual, DRV) ----------------
REG_POSE = 0x00F0           # X Y Z RX RY RZ，各 2 word，0.001 mm / 0.001 度
REG_MOTION = 0x00E0         # 0 = 停止
REG_OP_MODE = 0x0139        # 1 T1、2 T2、3 Auto
REG_ROBOT_ERR, REG_CTRL_ERR, REG_WARN = 0x01E0, 0x01FF, 0x020E
REG_RL_STATUS = 0x0213      # 0 關閉、1 執行中、2 中斷點、3 暫停、4 Pre-run
REG_RL_ID = 0x0220
REG_RL_LAST_LINE = 0x0227
REG_RL_MODE = 0x0228        # 1 Open、2 Run、3 Stop、6 Open & Run
REG_GO_CMD = 0x0300         # 寫 1000 = Motion Stop

# ---------------- 跟 path_runner.lua / loader.lua 的交握 ----------------
RL_VER = 1                  # = path_runner.lua 的 VER
LOADER_READY = 1001
BUF = 0x1000
PTS = 0x1020
REC = 16
MSG = 0x1F00
MAX_PTS = (MSG - 1 - PTS) // REC          # 237 點
LOAD_AREA = MSG - 1 - PTS
ST_RUN, ST_DONE, ST_TEST_DONE, ST_ERR = 2, 3, 6, 9

PASS_DISTANT, PASS_DEC = 0, 1

Pt = namedtuple("Pt", "x y z rx ry rz speed movp blend", defaults=(10.0, False, 1.0))
Pt.__doc__ = """一個路徑點
    x y z      mm (控制器目前座標系，跟 Modbus 讀到的一樣)
    rx ry rz   度
    speed      速度 % (0.1 ~ 100)
    movp       True = MovP (關節插補，路徑不是直線)；False = MovL (直線)
    blend      0 = 到這點停一下再走；> 0 = 不停直接接下一段
               (PASS_DEC 模式只看是否 > 0；PASS_DISTANT 模式 = 提早轉彎的距離 mm，1 ~ 100)"""


class SmoothPathError(RuntimeError):
    pass


def _dw(v):
    v = int(round(v * 1000)) & 0xFFFFFFFF
    return [v & 0xFFFF, v >> 16]


def _from_dw(lo, hi):
    return struct.unpack("<i", struct.pack("<HH", lo, hi))[0] / 1000.0


class DeltaArm:
    """最小的 Modbus 連線：讀寫暫存器 + 讀目前位置。已經有自己的連線類別的話，
    只要它有 _read(addr, n) -> list[int] 和 _write(addr, int | list[int]) 就能直接給 SmoothPath 用。"""

    def __init__(self, ip="192.168.1.1", port=502, unit=2, timeout=2):
        self.c = ModbusTcpClient(host=ip, port=port, timeout=timeout)
        if not self.c.connect():
            raise ConnectionError(f"連不到手臂 {ip}:{port}，請檢查網路設定")
        # pymodbus 3.6 用 slave=，3.10 起改成 device_id=
        params = inspect.signature(self.c.read_holding_registers).parameters
        self._uk = next((k for k in ("device_id", "slave", "unit") if k in params), "slave")
        self.unit = unit

    def _retry(self, fn, what):
        last = None
        for _ in range(3):
            try:
                if not self.c.connected:
                    self.c.connect()
                rr = fn()
                if not rr.isError():
                    return rr
                last = rr
            except Exception as e:
                last = e
            self.c.close()
            time.sleep(0.3)
        raise RuntimeError(f"{what} 失敗：{last}")

    def _read(self, addr, n):
        kw = {self._uk: self.unit}
        return self._retry(lambda: self.c.read_holding_registers(addr, count=n, **kw),
                           f"讀取 0x{addr:04X}").registers

    def _write(self, addr, values):
        kw = {self._uk: self.unit}
        if isinstance(values, list):
            return self._retry(lambda: self.c.write_registers(addr, values, **kw), f"寫入 0x{addr:04X}")
        return self._retry(lambda: self.c.write_register(addr, values, **kw), f"寫入 0x{addr:04X}")

    def pose(self):
        """[X, Y, Z, RX, RY, RZ] (mm / 度)"""
        r = self._read(REG_POSE, 12)
        return [_from_dw(r[i], r[i + 1]) for i in range(0, 12, 2)]

    def close(self):
        self.c.close()


class SmoothPath:
    """
    arm         DeltaArm 或任何有 _read/_write 的物件
    rl_id       控制器上 loader.lua 專案的 RL ID (install_loader.py 預設 900)
    pass_mode   PASS_DEC (預設，速度疊合，最順) 或 PASS_DISTANT (轉角磨圓，每段仍會加減速)
    overlap     PASS_DEC 的疊合 % (1 ~ 100)
    template    模板點編號：新點沿用它的 UF/TF 和手臂姿態 (Elbow/Shoulder/Flip)，點位表要先教好這一點
    slot0       暫存點起始編號：第 i 點會寫進 slot0 + i，這段點位表會被覆蓋
    """
    upload_chunk = 25           # 送 Lua 時一次寫幾個 word；控制器不收會自動改回 10

    def __init__(self, arm, rl_id=900, pass_mode=PASS_DEC, overlap=100, template=1, slot0=500):
        self.arm, self.rl_id = arm, rl_id
        self.pass_mode, self.overlap = pass_mode, overlap
        self.template, self.slot0 = template, slot0

    # ---------- 狀態 ----------
    def rl_status(self):
        return self.arm._read(REG_RL_STATUS, 1)[0]

    def diag(self):
        def rd(a):
            try:
                return self.arm._read(a, 1)[0]
            except Exception as e:
                return f"讀不到({e})"

        def hx(v):
            return f"0x{v:04X}" if isinstance(v, int) else v
        mode = rd(REG_OP_MODE)
        st = rd(REG_RL_STATUS)
        v = rd(BUF + 2)
        return (f"操作模式 {({1: 'T1', 2: 'T2', 3: 'Auto'}).get(mode, mode)}，"
                f"RL 狀態 {({0: '關閉', 1: '執行中', 2: '中斷點', 3: '暫停', 4: 'Pre-run'}).get(st, st)}，"
                f"RL ID {rd(REG_RL_ID)}，RL 最後行 {rd(REG_RL_LAST_LINE)}，"
                f"機器人錯誤 {hx(rd(REG_ROBOT_ERR))}，控制器錯誤 {hx(rd(REG_CTRL_ERR))}，警告 {hx(rd(REG_WARN))}，"
                f"0x1002 = {v} ({'載入器在等' if v == LOADER_READY else '程式在跑' if v == RL_VER else '?'})")

    def read_msg(self):
        n = min(self.arm._read(MSG - 1, 1)[0], 250)
        chars = []
        for i in range(0, n, 10):
            chars += self.arm._read(MSG + i, min(10, n - i))
        return bytes(c & 0xFF for c in chars).decode("utf-8", errors="replace")

    def _cmd(self, cmd, want, timeout):
        want = want if isinstance(want, tuple) else (want,)
        self.arm._write(MSG - 1, 0)
        self.arm._write(BUF, cmd)
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.arm._read(BUF, 1)[0]
            if st in want:
                return
            if st == ST_ERR:
                raise SmoothPathError(f"控制器回報錯誤：{self.read_msg() or '(沒有訊息)'}")
            time.sleep(0.02)
        raise SmoothPathError(f"等控制器回應逾時 (指令 {cmd}，0x1000 = {self.arm._read(BUF, 1)[0]})")

    # ---------- 送 Lua / 啟動 / 停止 ----------
    def upload_lua(self, path=LUA_PATH):
        """把 path_runner.lua 送給控制器上的 loader.lua (去掉整行註解，每個 word 放 2 個位元組)"""
        with open(path, "rb") as f:
            lines = [ln.strip() for ln in f.read().replace(b"\r\n", b"\n").split(b"\n")]
        src = b"\n".join(ln for ln in lines if ln and not (ln.startswith(b"--") and not ln.startswith(b"--[[")))
        t0 = time.time()
        self.arm._write(BUF, 23)
        time.sleep(0.1)
        for off in range(0, len(src), LOAD_AREA * 2):
            part = src[off:off + LOAD_AREA * 2]
            data = part + b"\0" if len(part) % 2 else part
            words = [data[i] | (data[i + 1] << 8) for i in range(0, len(data), 2)]
            i = 0
            while i < len(words):
                n = self.upload_chunk
                try:
                    self.arm._write(PTS + i, words[i:i + n])
                except RuntimeError:
                    if n <= 10:
                        raise
                    SmoothPath.upload_chunk = 10
                    continue
                i += n
            self.arm._write(BUF + 1, len(part))
            self._cmd(20, 21, 10.0)
        self.arm._write(BUF, 22)
        print(f"  已送出 path_runner.lua ({len(src)} bytes，{time.time() - t0:.1f} s)")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if self.arm._read(BUF + 2, 1)[0] == RL_VER:
                return
            if self.arm._read(BUF, 1)[0] == ST_ERR:
                raise SmoothPathError(f"path_runner.lua 執行失敗：{self.read_msg()}")
            time.sleep(0.05)
        raise SmoothPathError(f"path_runner.lua 沒跑起來 (0x1002 = {self.arm._read(BUF + 2, 1)[0]}，要 {RL_VER})")

    def _wait_ready(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            v = self.arm._read(BUF + 2, 1)[0]
            if v == RL_VER:
                return True
            if v == LOADER_READY:
                self.upload_lua()
                return True
            time.sleep(0.1)
        return False

    def start(self):
        """啟動 RL 並確保 path_runner.lua 在跑"""
        if self.rl_status() == 1:
            v = self.arm._read(BUF + 2, 1)[0]
            if v == RL_VER:
                return
            if v == LOADER_READY:
                self.upload_lua()
                return
            self.stop()
            time.sleep(0.3)
        self.arm._write(BUF + 2, 0)
        self.arm._write(BUF, 0)
        self.arm._write(REG_RL_ID, self.rl_id)
        for seq in ((2,), (6,), (1, 2)):                # Run → Open&Run → Open 再 Run
            for v in seq:
                self.arm._write(REG_RL_MODE, v)
                time.sleep(0.5)
            if self._wait_ready(4.0):
                return
        raise SmoothPathError(f"RL 沒有啟動 ({self.diag()})。請確認控制器在 Auto 模式、"
                              f"RL ID {self.rl_id} 是 loader.lua；不行就在 DRAStudio 按一次 Start 再試")

    def stop(self):
        """停掉 RL。RL 在跑的時候控制器不吃 Modbus GO 指令，所以走完一定要停"""
        self.arm._write(REG_RL_MODE, 3)
        t0 = time.time()
        while self.rl_status() == 1 and time.time() - t0 < 3.0:
            time.sleep(0.05)

    def abort(self):
        try:
            self.arm._write(REG_GO_CMD, 1000)
        finally:
            self.stop()

    # ---------- 執行 ----------
    def _write_points(self, points):
        if not points:
            raise SmoothPathError("路徑是空的")
        if len(points) > MAX_PTS:
            raise SmoothPathError(f"點數 {len(points)} 超過上限 {MAX_PTS}，請分段執行")
        regs = []
        for p in points:
            p = Pt(*p)
            if not 0 < p.speed <= 100:
                raise SmoothPathError(f"速度 {p.speed} 要在 0 ~ 100 %")
            for v in (p.x, p.y, p.z, p.rx, p.ry, p.rz):
                regs += _dw(v)
            regs += [int(round(p.speed * 10)), 1 if p.movp else 0, int(round(max(p.blend, 0) * 10)), 0]
        assert len(regs) == REC * len(points)
        for i in range(0, len(regs), 10):
            self.arm._write(PTS + i, regs[i:i + 10])
        self.arm._write(BUF + 1, len(points))
        self.arm._write(BUF + 10, [self.pass_mode, self.overlap, self.template, self.slot0])

    def self_test(self, points):
        """確認控制器讀到的點 = 寫進去的、RL 看到的目前位置 = Modbus 讀到的"""
        self._write_points(points)
        self._cmd(5, ST_TEST_DONE, 3.0)
        r = self.arm._read(0x1010, 12)
        echo = [_from_dw(r[i], r[i + 1]) for i in range(0, 6, 2)]
        rl_pose = [_from_dw(r[i], r[i + 1]) for i in range(6, 12, 2)]
        if max(abs(a - b) for a, b in zip(echo, Pt(*points[0])[:3])) > 0.01:
            raise SmoothPathError(f"控制器讀到的點 {echo} ≠ 寫入的 {Pt(*points[0])[:3]}")
        if hasattr(self.arm, "pose"):
            py = self.arm.pose()[:3]
            if max(abs(a - b) for a, b in zip(rl_pose, py)) > 1.0:
                raise SmoothPathError(f"RL 的目前位置 {rl_pose} ≠ Modbus 讀到的 {py}：座標系 (UF/TF) 不一致")
        return echo, rl_pose

    def run(self, points, timeout=600.0, verbose=True):
        """走完整條路徑才回傳。points：Pt 或 (x, y, z, rx, ry, rz[, speed, movp, blend]) 的 list"""
        self.start()
        try:
            self.self_test(points)
            self.arm._write(BUF + 3, 0)
            self._cmd(1, (ST_RUN, ST_DONE), 5.0)
            t0, last = time.time(), -1
            while True:
                st = self.arm._read(BUF, 1)[0]
                if st == ST_DONE:
                    break
                if st == ST_ERR:
                    raise SmoothPathError(f"執行路徑出錯：{self.read_msg() or '(沒有訊息)'}")
                if self.rl_status() != 1:
                    raise SmoothPathError(f"RL 停了 ({self.diag()})，請看教導器警報")
                if time.time() - t0 > timeout:
                    raise SmoothPathError("路徑執行逾時")
                i = self.arm._read(BUF + 3, 1)[0]
                if verbose and i != last:
                    last = i
                    print(f"\r  執行中：第 {i}/{len(points)} 點", end="", flush=True)
                time.sleep(0.05)
            while self.arm._read(REG_MOTION, 1)[0] != 0 and time.time() - t0 < timeout:
                time.sleep(0.02)
            if verbose:
                print(f"\r  ✅ 走完 {len(points)} 點，{time.time() - t0:.1f} s          ")
        except BaseException:
            if verbose:
                print()
            self.abort()
            raise
        self.stop()


# ---------------- 命令列測試 ----------------
def _main():
    import argparse
    import math
    ap = argparse.ArgumentParser(description="台達 DRV 連續路徑測試")
    ap.add_argument("mode", choices=["diag", "test", "square", "circle"])
    ap.add_argument("--ip", default="192.168.1.1")
    ap.add_argument("--unit", type=int, default=2, help="Modbus unit / slave id")
    ap.add_argument("--rl-id", type=int, default=900, help="loader.lua 的 RL ID (install_loader.py 預設裝在 900)")
    ap.add_argument("--speed", type=float, default=5.0, help="速度 %%")
    ap.add_argument("--size", type=float, default=30.0, help="方形邊長 / 圓形直徑 mm")
    ap.add_argument("--distant", action="store_true", help="改用 PASS DISTANT 模式比較")
    a = ap.parse_args()

    arm = DeltaArm(a.ip, unit=a.unit)
    sp = SmoothPath(arm, rl_id=a.rl_id, pass_mode=PASS_DISTANT if a.distant else PASS_DEC)
    x, y, z, rx, ry, rz = arm.pose()
    print(f"目前位置 X {x:.2f} Y {y:.2f} Z {z:.2f}  RX {rx:.2f} RY {ry:.2f} RZ {rz:.2f}")

    if a.mode == "diag":
        print(sp.diag())
        return
    if a.mode == "test":
        sp.start()
        try:
            echo, rl_pose = sp.self_test([Pt(x, y, z, rx, ry, rz, a.speed)])
            print(f"✅ 自我測試通過：控制器讀到 {echo}，RL 目前位置 {rl_pose}")
        finally:
            sp.stop()
        return

    s = a.size
    blend = 3.0 if a.distant else 1.0
    if a.mode == "square":
        corners = [(x + s, y), (x + s, y + s), (x, y + s), (x, y)]
        pts = [Pt(cx, cy, z, rx, ry, rz, a.speed, False, blend if k < 3 else 0)
               for k, (cx, cy) in enumerate(corners)]
    else:
        r, n = s / 2, 40
        pts = [Pt(x - r + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n), z,
                  rx, ry, rz, a.speed, False, blend if k < n else 0) for k in range(1, n + 1)]
    input(f"手臂會在目前高度走{'方形' if a.mode == 'square' else '圓形'} {s:.0f} mm (速度 {a.speed}%)，"
          "請確認周圍淨空。按 Enter 開始，Ctrl+C 取消")
    sp.run(pts)


if __name__ == "__main__":
    try:
        _main()
    except SmoothPathError as e:
        sys.exit(f"❌ {e}")
