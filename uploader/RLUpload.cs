// 不用 DRAStudio，用台達官方 API (RABD.Lib.dll，來自 DRAStudio 的 API/DROE_API_CS.zip) 上傳 RL 專案。
// 由 install_loader.py 自動編譯並呼叫，一般不用直接執行。
//
//   RLUpload.exe <指令> <控制器IP> <電腦IP> <電腦埠> <手臂型號> [參數...]
//     手臂型號：VA (DRV 六軸) / Scara / ScaraThree / ScaraFive
//     指令：
//       list                        列出控制器上的專案 (只讀)
//       upload   <RL ID> <資料夾>    上傳專案 (會覆蓋同一個 RL ID)
//       download <專案名稱> <資料夾>  把控制器上的專案下載回來 (只讀，備份用)
//
// 上傳時控制器會反過來連到「電腦IP:電腦埠」拿檔案，所以電腦 IP 要是控制器網段上的那張網卡，
// 而且 Windows 防火牆要允許 RLUpload.exe 接受連線。
using System;
using System.IO;
using System.Reflection;
using RABD.Lib;
using RABD.DROE.SystemDefine;

class RLUpload
{
    static string LibDir;

    static int Main(string[] args)
    {
        LibDir = Path.Combine(Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location), "lib");
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string f = Path.Combine(LibDir, new AssemblyName(e.Name).Name + ".dll");
            return File.Exists(f) ? Assembly.LoadFrom(f) : null;
        };
        Directory.SetCurrentDirectory(LibDir);     // API 會從目前資料夾讀 Alarm_*.xml、寫 logs
        try
        {
            return Run(args);
        }
        catch (Exception ex)
        {
            Console.WriteLine("ERROR " + ex.GetType().Name + ": " + ex.Message);
            return 2;
        }
    }

    static int Run(string[] args)
    {
        if (args.Length < 5)
        {
            Console.WriteLine("usage: RLUpload <list|upload|download> <robotIP> <pcIP> <pcPort> <VA|Scara|...> [args]");
            return 1;
        }
        string cmd = args[0], ip = args[1], pcIp = args[2];
        int port = int.Parse(args[3]);
        eRobotType type = (eRobotType)Enum.Parse(typeof(eRobotType), args[4], true);

        Robot d = new Robot();
        if (!d.ConnectRobot(ip, pcIp, port, type))
        {
            Console.WriteLine("ERROR connect " + ip + " (pc " + pcIp + ":" + port + ", " + type + ") failed");
            return 3;
        }
        Console.WriteLine("CONNECTED " + ip + " pc " + pcIp + ":" + port + " type " + type);
        try
        {
            switch (cmd)
            {
                case "list":
                    foreach (string p in d.ProjectList())
                        Console.WriteLine("PROJECT " + p);
                    return 0;
                case "upload":
                {
                    int id = int.Parse(args[5]);
                    string dir = Path.GetFullPath(args[6]);
                    bool ok = d.ProjectUploadToController(id, dir);
                    Console.WriteLine(ok ? "OK upload" : "ERROR upload returned false");
                    return ok ? 0 : 4;
                }
                case "download":
                {
                    bool ok = d.ProjectDownloadToLocal(args[5], Path.GetFullPath(args[6]));
                    Console.WriteLine(ok ? "OK download" : "ERROR download returned false");
                    return ok ? 0 : 4;
                }
                default:
                    Console.WriteLine("ERROR unknown command " + cmd);
                    return 1;
            }
        }
        finally
        {
            d.DisConnectRobot();
        }
    }
}
