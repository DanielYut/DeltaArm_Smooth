# delta_smooth：台達 DRV 手臂連續路徑（不停頓）

用 Modbus GO 指令（0x0300 = 301/302）控制手臂，一次只能送一個點，**每到一個點都會減速停下**，走曲線或多邊形時會一頓一頓的。

這個套件把整條路徑寫進控制器，由控制器上的 Lua（RL）程式用 `MovL + PASS()` 連續走完，點和點之間不會停。你只要在 Python 給一串點就好，**不用學 Lua，也不用每次開 DRAStudio**。

已在 DRV + DCV 控制器（MS2 機器人語言）上測試過。

## 檔案

| 檔案 | 放在哪 | 用途 |
|---|---|---|
| `loader.lua` | 控制器（裝**一次**） | 載入器：接收 Python 送來的 Lua 程式並執行 |
| `path_runner.lua` | 你的電腦 | 真正走路徑的程式，Python 每次會自動送進控制器 |
| `delta_smooth.py` | 你的電腦 | Python 介面：`DeltaArm`（連線）、`SmoothPath`（執行路徑）、`Pt`（路徑點） |
| `install_loader.py` | 你的電腦 | **不用 DRAStudio**，直接把 `loader.lua` 裝進控制器 |
| `uploader/` | 你的電腦 | 上傳工具 `RLUpload.exe` 和台達官方 API 的 DLL（來自 DRAStudio 安裝資料夾的 `API/DROE_API_CS.zip`） |

## 第一次設定（每台控制器做一次，**不需要 DRAStudio**）

1. 電腦接上控制器的網路（預設控制器 IP 是 `192.168.1.1`，電腦要設成同一個網段，例如 `192.168.1.30`）。
2. 安裝 Python 套件：`pip install pymodbus`（3.6 ~ 3.11 都可以）。
3. **先看控制器上有哪些專案**（只讀，不會改任何東西）：
   ```
   python install_loader.py --list
   ```
4. **裝載入器**，預設裝在 RL ID **900**：
   ```
   python install_loader.py
   ```
   - 如果 900 號已經有專案，程式會先問你要不要覆蓋。不想覆蓋就用 `--rl-id` 換一個沒用到的編號，之後跑 `delta_smooth.py` 和 `SmoothPath(rl_id=...)` 都要用同一個編號。
   - 這一步只能在 **Windows** 上做（台達 API 是 .NET Framework），Windows 10/11 不用另外安裝任何東西。
   - 也可以改用 DRAStudio：開一個 RL 專案，把 `loader.lua` 整段貼進 `main.lua`，再下載到控制器。
5. **教一個模板點 P1**（用教導器或 DRAStudio 都可以）：點位表的 **P1** 一定要存在。新的路徑點會沿用 P1 的**座標系（UF/TF）和手臂姿態（Elbow/Shoulder/Flip）**，所以請用你平常的工作姿態和座標系來教。
6. **空出暫存點**：點位表 **501 號以後**會被當成暫存點覆蓋，不要在那裡放教點。起點可以用 `slot0` 改。
7. 控制器切到 **Auto 模式**，然後 Servo ON。

完成後，**換電腦、重開電腦都不用再做**，只要跑 Python 就好。

## 測試

```
python delta_smooth.py diag      # 讀控制器狀態（手臂不動）
python delta_smooth.py test      # 送 Lua + 自我測試（手臂不動）
python delta_smooth.py circle    # 在目前高度畫直徑 30 mm 的圓（會動！）
python delta_smooth.py square    # 在目前高度畫 30 mm 方形（會動！）
```

可加的參數：`--ip 192.168.1.1`、`--unit 2`（Modbus slave id）、`--rl-id 900`、`--speed 5`、`--size 30`，以及 `--distant`（改用 DISTANT 模式，可以比較兩種模式的差別）。

> ⚠️ `circle` / `square` 會讓手臂在**目前高度**移動，請先把手臂移到空中、確認周圍淨空，並準備好急停。

## 在自己的程式裡用

```python
from delta_smooth import DeltaArm, SmoothPath, Pt

arm = DeltaArm("192.168.1.1", unit=2)
x, y, z, rx, ry, rz = arm.pose()          # 目前位置 (mm / 度)

path = [
    Pt(x + 50, y,      z, rx, ry, rz, speed=10),           # 直線過去，不停
    Pt(x + 50, y + 50, z, rx, ry, rz, speed=10),
    Pt(x,      y + 50, z, rx, ry, rz, speed=10),
    Pt(x,      y,      z, rx, ry, rz, speed=10, blend=0),  # 最後一點：到位停下
]
SmoothPath(arm, rl_id=900).run(path)        # 走完才回傳；出錯會停手臂並丟 SmoothPathError
```

### `Pt(x, y, z, rx, ry, rz, speed=10, movp=False, blend=1)`

| 欄位 | 說明 |
|---|---|
| `x y z` | mm，跟 Modbus 讀到的座標系一樣 |
| `rx ry rz` | 度，每個點可以不一樣 |
| `speed` | 速度 %（0.1 ~ 100） |
| `movp` | `False` = MovL（走直線）；`True` = MovP（關節插補，比較快，但路徑不是直線） |
| `blend` | `0` = 到這點停一下；`> 0` = 不停，直接接下一段。最後一點一律會停 |

### `SmoothPath(arm, rl_id=900, pass_mode=PASS_DEC, overlap=100, template=1, slot0=500)`

| 參數 | 說明 |
|---|---|
| `pass_mode` | `PASS_DEC`（預設）：前一段的減速和下一段的加速重疊，**速度連續、最順**。`PASS_DISTANT`：只在點附近提早 `blend` mm 轉彎，每段仍會加減速，點很密時會頓 |
| `overlap` | DEC 模式重疊的 %，100 = 完全重疊 |
| `template` | 模板點編號（預設 P1） |
| `slot0` | 暫存點起始編號（第 i 點寫到 `slot0 + i`） |

已經有自己的 Modbus 連線類別的話，只要它有 `_read(addr, n) -> list[int]` 和 `_write(addr, int 或 list[int])`，就可以直接傳給 `SmoothPath`，不必用 `DeltaArm`。

## 一定要知道的事

- **RL 在執行時，控制器不接受 Modbus GO 指令。** `run()` 走完會自動停掉 RL，之後就能照常用 GO 指令。如果你在 DRAStudio 手動按了 Start，要記得按 Stop。
- **一次最多 237 個點**，超過請分段呼叫 `run()`。
- **速度越快，DEC 模式的轉角切得越往內側**。需要貼角時，降低速度，或把轉角那一點設成 `blend=0`。
- 點的座標會先在控制器讀回比對，對不上就報錯、不會移動。另外會先做一次自我測試，確認 RL 看到的目前位置跟 Modbus 一致。
- 想改 Lua 行為，直接改 `path_runner.lua`，存檔後再跑 Python 就生效。如果改了 `VER`，`delta_smooth.py` 的 `RL_VER` 要改成一樣。錯誤訊息裡的行號是去掉註解行之後的行號。
- RL 自動啟動失敗（出現「RL 沒有啟動」）時：確認是 Auto 模式、RL ID 正確；還是不行就在 DRAStudio 按一次 Start 再試。

## 原理（給想改的人）

- **用點位編號而不是自己組的 table**：控制器的 `MovL` / `MovP` 只接受點位表編號。所以先 `CopyPoint(暫存編號, 模板)`，再用 `WritePoint` 改 X/Y/Z/RX/RY/RZ。
- **先把所有點寫好再開始動**：手臂動起來之後，迴圈裡只剩 `MovL`，下一段指令才來得及在轉彎前送出。
- **`PassMode("DEC")` + `SetOverlapTime(100)`**：手冊 1.7 節。DISTANT 模式下每段仍會加減速，DEC 模式讓速度曲線疊在一起。
- **透過 Modbus 傳 Lua**：Python 把 Lua 原始碼寫進 Device Buffer（0x1020 起，每個 word 放 2 個位元組），`loader.lua` 收齊後用 `load()` 執行。各暫存器的交握細節見 `path_runner.lua` 檔頭。
