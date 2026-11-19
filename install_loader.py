"""
不用 DRAStudio，把 loader.lua 裝進控制器 (每台控制器只要做一次)。

用台達官方 API (uploader/lib 裡的 RABD.Lib.dll，來自 DRAStudio 安裝資料夾的 API/DROE_API_CS.zip)，
由 uploader/RLUpload.exe 上傳。RLUpload.exe 不見了會用 Windows 內建的 C# 編譯器自動重新編譯。
只能在 Windows 上執行。

    python install_loader.py --list                  只列出控制器上的專案 (不改任何東西)
    python install_loader.py                         裝到 RL ID 900 (該編號已有專案會停下來問)
    python install_loader.py --rl-id 5               指定 RL ID
    加 --ip 192.168.1.1 指定控制器 IP

裝好之後：python delta_smooth.py test --rl-id 900
"""
import argparse
import glob
import os
import shutil
import socket
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
UP = os.path.join(HERE, "uploader")
EXE = os.path.join(UP, "RLUpload.exe")
TEMPLATE = os.path.join(UP, "template")
LOADER = os.path.join(HERE, "loader.lua")
PROJECT_NAME = "SmoothLoader"


def build_exe():
    csc = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Microsoft.NET", "Framework", "v4.0.30319", "csc.exe")
    if not os.path.exists(csc):
        sys.exit(f"找不到 C# 編譯器 {csc}，也沒有 RLUpload.exe。請跟原作者要編譯好的 RLUpload.exe")
    print("編譯 RLUpload.exe ...")
    r = subprocess.run([csc, "-nologo", "-platform:x86", "-out:" + EXE,
                        "-r:" + os.path.join(UP, "lib", "RABD.Lib.dll"),
                        "-r:" + os.path.join(UP, "lib", "RABD.DROE.SystemDefine.dll"), os.path.join(UP, "RLUpload.cs")],
                       capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("編譯失敗：\n" + r.stdout + r.stderr)


CONN = {}                   # ip / pc_ip / port / type，main() 設定


def local_ip_for(target):
    """電腦連到控制器時用的那張網卡的 IP (控制器上傳時會反過來連這個 IP)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target, 502))
        return s.getsockname()[0]
    finally:
        s.close()


def call(cmd, *args, timeout=180):
    if not os.path.exists(EXE):
        build_exe()
    r = subprocess.run([EXE, cmd, CONN["ip"], CONN["pc_ip"], str(CONN["port"]), CONN["type"], *args],
                       capture_output=True, text=True, timeout=timeout, encoding="utf-8", errors="replace")
    out = (r.stdout or "") + (r.stderr or "")
    return r.returncode, out


def list_projects(ip):
    code, out = call("list")
    if code != 0:
        sys.exit(f"讀不到控制器專案列表 (請確認 IP、網路、控制器開機)：\n{out}")
    return [ln[len("PROJECT "):].strip() for ln in out.splitlines() if ln.startswith("PROJECT ")]


def project_id(name):
    """控制器回傳的名稱前 4 碼是編號 (台達範例的寫法)"""
    try:
        return int(name[:4])
    except ValueError:
        return None


def make_project(dst):
    """用模板組一個 DRAStudio 專案資料夾，main.lua = loader.lua"""
    os.makedirs(dst)
    for f in glob.glob(os.path.join(TEMPLATE, "*")):
        name = os.path.basename(f)
        if name.startswith("project."):
            name = PROJECT_NAME + name[len("project"):]
        shutil.copy(f, os.path.join(dst, name))
    shutil.copy(LOADER, os.path.join(dst, "main.lua"))


def main():
    ap = argparse.ArgumentParser(description="不用 DRAStudio 把 loader.lua 裝進台達控制器")
    ap.add_argument("--ip", default="192.168.1.1")
    ap.add_argument("--rl-id", type=int, default=900, help="要裝到哪個 RL ID (0~999，預設 900)")
    ap.add_argument("--list", action="store_true", help="只列出控制器上的專案")
    ap.add_argument("--yes", action="store_true", help="RL ID 已有專案時不問，直接覆蓋")
    ap.add_argument("--pc-ip", help="電腦在控制器網段上的 IP (預設自動偵測)")
    ap.add_argument("--port", type=int, default=11000, help="控制器回連電腦用的埠 (台達範例用 11000)")
    ap.add_argument("--type", default="VA", choices=["VA", "Scara", "ScaraThree", "ScaraFive"],
                    help="手臂型號：DRV 六軸 = VA (預設)")
    a = ap.parse_args()
    if os.name != "nt":
        sys.exit("只能在 Windows 執行 (台達 API 是 .NET Framework)")
    CONN.update(ip=a.ip, pc_ip=a.pc_ip or local_ip_for(a.ip), port=a.port, type=a.type)
    print(f"控制器 {a.ip}，電腦 {CONN['pc_ip']}:{a.port}，型號 {a.type}")

    projects = list_projects(a.ip)
    print(f"控制器 {a.ip} 上的專案 ({len(projects)} 個)：")
    for p in projects:
        print("  ", p)
    if a.list:
        return

    same = [p for p in projects if project_id(p) == a.rl_id]
    if same and not a.yes:
        ans = input(f"⚠️ RL ID {a.rl_id} 已經有專案 {same[0]}，上傳會把它覆蓋掉。確定要覆蓋？(y/n) > ")
        if ans.strip().lower() != "y":
            sys.exit("取消。可以用 --rl-id 換一個沒用到的編號")

    with tempfile.TemporaryDirectory() as tmp:
        proj = os.path.join(tmp, PROJECT_NAME)
        make_project(proj)
        print(f"上傳 loader.lua 到 RL ID {a.rl_id} ...")
        # 台達 API 的上傳/下載就算成功也常回傳 false (實測)，所以不看回傳值，改成下載回來比對內容
        code, out = call("upload", str(a.rl_id), proj)
        if code not in (0, 4):
            print(out.strip())
            sys.exit("❌ 上傳時出錯")

        after = list_projects(a.ip)
        hit = [p for p in after if project_id(p) == a.rl_id]
        if not hit:
            sys.exit(f"❌ 上傳失敗：控制器上沒有 RL ID {a.rl_id} 的專案。上傳時控制器會反過來連到電腦 "
                     f"{CONN['pc_ip']}:{a.port}：請確認 Windows 防火牆允許 RLUpload.exe (第一次執行若跳出防火牆"
                     "視窗請按「允許」)，以及電腦 IP 是控制器網段上的那張網卡 (可用 --pc-ip 指定)")

        back = os.path.join(tmp, "verify")
        os.makedirs(back)
        call("download", hit[0], back)
        got = glob.glob(os.path.join(back, "**", "main.lua"), recursive=True)
        with open(LOADER, "rb") as f:
            want = f.read()
        if not got:
            sys.exit(f"❌ 控制器上有 {hit[0]}，但下載不回來驗證。可以直接試：python delta_smooth.py test --rl-id {a.rl_id}")
        with open(got[0], "rb") as f:
            if f.read() != want:
                sys.exit(f"❌ 控制器上 {hit[0]} 的 main.lua 跟 loader.lua 不一樣，上傳沒成功，請再試一次")

    print(f"✅ 上傳完成並驗證：RL ID {a.rl_id} = {hit[0]}，main.lua 跟 loader.lua 完全相同")
    print(f"下一步 (手臂不會動)：python delta_smooth.py test --rl-id {a.rl_id}")


if __name__ == "__main__":
    main()
