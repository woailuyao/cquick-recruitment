#打开摄像头预览，按r开始或停止录像，按s保存当前帧，按q或Esc退出
#--device选摄像头编号，--duration 60表示打开后自动录60秒
#视频和截图默认存在videos/raw，文件名带时间，后面可以拿去标注

#兼容开关：让下面函数签名里写的类型注解只当文字看，旧版本 Python 也不报错
from __future__ import annotations

import argparse#解析命令行参数（--duration 60 这种）
import os#判断当前是不是 Windows
import time#取时间、计时
from pathlib import Path#更省心的路径对象，能自动拼目录

import cv2#计算机视觉库：打开摄像头、读写视频、显示窗口

#__file__ 是当前这个脚本的路径，resolve() 变成绝对路径，parents[2] 往上数两级
#从 src/soccer_vision/collect_video.py 往上两级正好是项目根目录
PROJECT_ROOT = Path(__file__).resolve().parents[2]


def stamp() -> str:#生成形如 20261110_204512 的时间串，用作文件名
    return time.strftime("%Y%m%d_%H%M%S")#年-月-日_时-分-秒


def main(argv=None) -> int:#主函数：整个脚本的流程都在这里面
    ap = argparse.ArgumentParser(description="摄像头录视频 / 截图")#创建命令行参数解析器
    #默认 -1：自动从 0 试到 5，哪个能出帧就用哪个；想固定就写 --device 0
    ap.add_argument("--device",type=int,default=-1,help="摄像头编号；-1 表示自动探测，默认自动")
    #后端：auto 先 DSHOW 再 MSMF 再 ANY，某个后端起不来时会自动换下一个
    ap.add_argument("--backend",choices=("auto","dshow","msmf","any"),default="auto",help="摄像头后端，默认 auto")
    ap.add_argument("--width",type=int,default=1280)#期望的视频宽度
    ap.add_argument("--height",type=int,default=720)#期望的视频高度
    ap.add_argument("--fps",type=int,default=30)#期望的帧率：1 秒录多少张图
    ap.add_argument("--duration",type=float,default=0.0,help="自动录多少秒，0 表示手动按 r 控制")
    ap.add_argument("--out-dir",default=str(PROJECT_ROOT / "videos/raw"))#文件存到哪，默认项目根的 videos/raw
    args = ap.parse_args(argv)#真正去读命令行；没传就全用默认值

    out_dir = Path(args.out_dir)#把字符串路径变成 Path 对象
    out_dir.mkdir(parents=True,exist_ok=True)#创建目录；父目录一起建；已存在也不报错

    #先在候选设备上试打开并读几帧，能出帧才算真可用
    def try_open(device, backend):#试开一个「编号+后端」组合
        cap = cv2.VideoCapture(device, backend)#按编号和后端打开
        if not cap.isOpened():#打不开就直接放弃
            cap.release()#立刻释放，避免占住后面的尝试
            return None#None 表示这个组合不行
        for _ in range(3):#连读 3 帧，USB 相机头几帧常是空的
            cap.read()#读掉
        ok, frame = cap.read()#再读一帧做判定
        if not ok or frame is None:#读不到图
            cap.release()#释放
            return None#这个组合也不行
        return cap#能出帧，返回已打开的句柄

    backend_map = {#后端名字到 OpenCV 常量的映射
        "dshow": (cv2.CAP_DSHOW, "DSHOW"),#Windows 老接口，打开快
        "msmf": (cv2.CAP_MSMF, "MSMF"),#Windows 新接口，兼容性好
        "any": (cv2.CAP_ANY, "ANY"),#让 OpenCV 自己挑
    }
    if args.backend == "auto":#auto 就按顺序都试一遍
        backends = [backend_map["dshow"], backend_map["msmf"], backend_map["any"]]#三种后端依次试
    else:#指定了后端就只用它
        backends = [backend_map[args.backend]]#只保留一个候选

    devices = [args.device] if args.device >= 0 else list(range(0, 6))#没指定编号就试 0-5
    cap = None#最终句柄先置空
    used_device = args.device#记录最后用了哪个编号
    used_backend = ""#记录最后用了哪个后端
    for device in devices:#外层循环：逐个编号
        for backend, backend_name in backends:#内层循环：逐个后端
            print(f"[probe] 试 设备 {device} / {backend_name} …")#告诉用户正在试哪个
            cap = try_open(device, backend)#真正去开
            if cap is not None:#开出帧了
                used_device = device#记下编号
                used_backend = backend_name#记下后端
                break#跳出内层循环
        if cap is not None:#已经成功就不试后面的编号
            break#跳出外层循环

    if cap is None:#所有组合都没成
        print("[error] 所有设备/后端都打不开摄像头")#明确报错
        print("        检查：USB 线是否插紧、相机是否被 OBS/微信/钉钉占用、设备管理器有无黄色感叹号")#给排查方向
        return 2#返回非 0 表示失败
    print(f"[cam] 使用 设备 {used_device} / {used_backend}")#告诉用户实际用的是哪个

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,args.width)#请求摄像头改成这个宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,args.height)#请求改成这个高度
    cap.set(cv2.CAP_PROP_FPS,args.fps)#请求这个帧率
    #cap.get 取摄像头实际给的分辨率；`or args.width` 是兜底：万一是 0 就用我们期望的值
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or args.width)
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or args.height)
    print(f"[cam] 摄像头 {used_device}（{used_backend}）  {real_w}x{real_h}")#打印实际生效的编号与后端
    if (real_w,real_h) != (args.width,args.height):#元组比较：实际和期望不一样
        print(f"[warn] 实际分辨率 {real_w}x{real_h} 与期望 {args.width}x{args.height} 不同，标定要用实际分辨率")

    #预热：很多 USB 相机刚打开时前十几帧是灰的或全黑的，
    #这里先空读一批丢掉，避免录出来的第一秒是灰屏
    for _ in range(15):#空读 15 帧，不处理
        cap.read()#读掉就扔
    ok_probe,probe = cap.read()#再读一帧当样本
    if ok_probe and probe is not None and float(probe.std()) < 3.0:#标准差极小 = 几乎纯色
        print("[warn] 抓到的是纯灰/纯黑画面：先确认镜头盖摘了、相机没被 OBS 虚拟摄像头占用")

    win = f"collect_video  设备{used_device}/{used_backend}  [r]录制 [s]截图 [q]退出"#窗口标题带上实际设备
    fps_nominal = float(cap.get(cv2.CAP_PROP_FPS) or args.fps)#摄像头报的帧率，取不到就用参数
    writer = None#录像对象：None 表示当前没在录
    rec_start = 0.0#这次录制开始的时间点
    frames = 0#这次一共录了多少帧
    shot_count = 0#截图计数，避免同一秒多次截图重名
    auto = args.duration > 0#是否一打开就自动录
    gray_streak = 0#连续多少帧几乎是纯色（判断相机是不是没出图）
    if auto:#自动模式下先提示一句
        print(f"[rec] 自动录制 {args.duration:.0f} 秒 …")

    #打开即开始录（--duration 模式）
    def start_rec():#内部小函数：开始录制
        nonlocal writer,rec_start,frames#声明这几个变量来自外层函数，改的是外层那个
        path = out_dir / f"cam_{stamp()}.mp4"#用当前时间当文件名，避免覆盖
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")#指定视频编码格式 mp4v（星号把字符串拆成 4 个字符）
        #VideoWriter(保存路径, 编码, 帧率, (宽, 高))：帧率最低给 10，太低了播放会卡
        writer = cv2.VideoWriter(str(path),fourcc,max(fps_nominal,10.0),(real_w,real_h))
        rec_start = time.perf_counter()#记下开始时刻（perf_counter 精度高，适合计时）
        frames = 0#帧数清零
        print(f"[rec] 开始 -> {path}  ({fps_nominal:.0f} fps)")

    #预览时摄像头持续送来图片，录制时还要把这些图片按顺序写进视频
    #停止录制要让视频写入器收尾，这样同一台摄像头随后还能继续预览或录制下一段
    def stop_rec():#内部小函数：停止录制
        nonlocal writer#同样声明要改外层的 writer
        if writer is not None:#确实在录才停
            writer.release()#关闭视频文件，把数据真正写进磁盘
            dur = time.perf_counter() - rec_start#录制时长（秒）
            #实际帧率 = 总帧数 / 时长；`if dur > 0 else 0` 防止除以 0 报错
            print(f"[rec] 停止：{frames} 帧 / {dur:.1f} s  ({frames / dur if dur > 0 else 0:.1f} fps 实际)")
            writer = None#置空，表示现在没在录了

    if auto:#自动模式：一进来就开录
        start_rec()

    try:#try 包住主循环，保证最后一定能收尾
        while True:#无限循环，直到按键退出或读帧失败
            ok,frame = cap.read()#读一帧：ok 是成功没，frame 是这一帧图像
            if not ok:#读不到
                print("[warn] 读帧失败，可能是摄像头被别的程序抢了")
                break#跳出循环

            #灰帧检测：连续半秒都是几乎没变化的纯色，说明相机没真正出图
            if float(frame.std()) < 2.0:#标准差极小 = 画面几乎是纯色
                gray_streak += 1#累计
                if gray_streak == 30:#攒够 30 帧（约 1 秒）提醒一次
                    print("[warn] 画面几乎是纯色：检查镜头盖、USB 供电，或换 --device 1 再试")
            else:#正常画面
                gray_streak = 0#清零

            if writer is not None:#正在录制
                writer.write(frame)#把这一帧写进视频文件
                frames += 1#帧数加一
                if auto and time.perf_counter() - rec_start >= args.duration:#自动模式且到了时长
                    print("[rec] 到时间了，自动停止")
                    stop_rec()#停止录制
                    break#退出循环

            view = frame.copy()#复制一份再画字，避免污染要保存的原图
            h = view.shape[0]#shape 是 (高, 宽, 通道数)，[0] 取高度
            if writer is not None:#录制中：左上角画红点 + 时长
                cv2.circle(view,(24,24),9,(0,0,255),-1)#圆心(24,24)、半径9、红色、-1表示实心填充
                #putText(图, 文字, 左下角坐标, 字体, 字号, 颜色 BGR, 线宽)
                cv2.putText(view,f"REC {time.perf_counter() - rec_start:5.1f}s",(42,32),
                            cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
            else:#没在录：显示绿字提示按 r
                cv2.putText(view,"IDLE  press r",(16,32),
                            cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,220,0),2)
            #左下角显示分辨率，h-14 让它贴近底边；颜色 (200,200,200) 是浅灰
            cv2.putText(view,f"{real_w}x{real_h}",(16,h - 14),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(200,200,200),1)

            cv2.imshow(win,view)#把画好的画面显示在窗口里
            key = cv2.waitKey(1) & 0xFF#等 1 毫秒看有没有按键；&0xFF 只保留低 8 位
            if key in (27,ord("q")):#27 是 ESC 的键码；ord("q") 把字符 q 变成键码
                break#退出
            if key == ord("r"):#按了 r
                if writer is None:#没在录就开录
                    start_rec()
                else:#正在录就停
                    stop_rec()
            elif key == ord("s"):#按了 s：截图
                shot_count += 1#计数加一
                p = out_dir / f"shot_{stamp()}_{shot_count:02d}.png"#:02d 表示补成两位数字，如 01
                cv2.imwrite(str(p),frame)#把这一帧存成 png 图片
                print(f"[shot] {p}")
    except KeyboardInterrupt:#用户在终端按了 Ctrl+C
        print("\n[cam] Ctrl+C")
    finally:#finally：不管上面怎么退出，这段一定执行
        if writer is not None:#还在录就先保存
            stop_rec()
        cap.release()#释放摄像头，别的程序才能用
        try:#关闭所有窗口可能出错，单独兜住
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001              # 任何异常都忽略
            pass#什么都不做

    print(f"[done] 输出目录 {out_dir}")#告诉用户文件在哪
    return 0#返回 0 表示正常结束


#这个判断的意思是：只有直接运行这个文件才执行下面的 main()；
#如果是被别的文件 import 进来，就不要自动跑（避免意外启动摄像头）
if __name__ == "__main__":
    raise SystemExit(main())#用 main 的返回值作为程序的退出码
