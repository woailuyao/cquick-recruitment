
import argparse#标准库：解析命令行参数（--source 之类）
import math#勾股、无穷
import os#建目录
import sys#修改模块搜索路径
import time#计时算 FPS
from pathlib import Path#路径对象，比字符串好用

import numpy as np#数组与随机数
import cv2#显示窗口、写视频

#允许 `python src/soccer_vision/run_demo.py` 与 `python -m soccer_vision.run_demo` 两种跑法
#__file__ 是当前这个文件的路径；parents[1] 就是 src 目录（parents[0] 是 soccer_vision）。
_SRC = Path(__file__).resolve().parents[1]

if str(_SRC) not in sys.path:#如果 src 还没在搜索路径里
    sys.path.insert(0,str(_SRC))#插到最前面，这样 import 才能找到包

# noqa: E402 是给代码检查工具的注释：这两行 import 故意写在 sys.path 之后。
from soccer_vision.configio import load_extrinsics, load_intrinsics, load_runtime_config  # noqa: E402
from soccer_vision.detect import Detection, YoloDetector  # noqa: E402
from soccer_vision.field_map import FieldMap, Observation, estimate_pose  # noqa: E402
from soccer_vision.geometry import (  # noqa: E402
    GroundProjector,
    Pose2D,
    build_extrinsics,
    project_ground_to_pixel,
    wrap_deg,
)

from soccer_vision.localizer import Localizer  # noqa: E402
from soccer_vision.video_source import FrameSource  # noqa: E402
from soccer_vision.visualize import MapCanvas, compose, draw_left  # noqa: E402
from soccer_interfaces.pose_types import FieldPose  # noqa: E402
from soccer_interfaces.ros2_stub import make_publisher  # noqa: E402


#合成自检用的示例相机：1280x720、fx=fy=900、无畸变、装在 0.45 m 高、俯角 20°
#内参矩阵：fx/fy 是焦距（像素），640/360 是图像中心（主点）。
DEMO_K = np.array([[900.0,0.0,640.0],[0.0,900.0,360.0],[0.0,0.0,1.0]],dtype=float)
DEMO_DIST = np.zeros(5,dtype=float)#5 个畸变系数全 0 = 没有畸变
DEMO_HEIGHT_M = 0.45#相机离地 45 厘米
DEMO_PITCH_DEG = 20.0#镜头往下倾斜 20 度


#--------------------------------------------------------------------------- #
#合成数据自检
#--------------------------------------------------------------------------- #
def _project_map_point(pose: Pose2D,feature_xy,K,dist,R,t):
    #把地图点投到画面上，返回是否可见、像素坐标和相机系深度
    #先把地图上的点换算到载体坐标系（相对于相机所在的载体）。
    base_xy = pose.inverse_apply([[feature_xy[0],feature_xy[1]]])[0]
    p_base = np.array([[base_xy[0],base_xy[1],0.0]],dtype=float)#补上 z=0（在地面上）
    p_cam = (p_base - t) @ R#p_cam = R^T (p_base - t)

    if p_cam[0,2] <= 0.05:#深度小于 5 厘米 = 在相机背后，看不到
        return False,None,0.0
    uv = project_ground_to_pixel(K,dist,R,t,p_base)[0]#投影成像素
    w,h = int(K[0,2] * 2),int(K[1,2] * 2)#由主点推出图像尺寸
    inside = 0.0 <= uv[0] < w and 0.0 <= uv[1] < h#判断是否落在画面里
    return inside,uv,float(p_cam[0,2])#顺便返回深度


#自检先给相机安排一个已知位置，算出它应该看到哪些标志，再让定位算法根据观测猜位置。
#把猜出的位姿与预设答案比较，就能分别检查位置和朝向误差。
def synthetic_self_test(field_map: FieldMap,trials: int = 60,noise_px: float = 2.0,seed: int = 0) -> bool:
    #用已知位姿生成像素点，再投回地面，检查算出的位姿和真值差多少
    print("=" * 66)#字符串乘数字 = 重复 66 次
    print("合成数据自检（不需要视频 / 权重 / 标定）")
    print(f"  地图：{field_map.source}  共 {len(field_map)} 个特征点 {field_map.counts()}")
    R,t = build_extrinsics(DEMO_HEIGHT_M,DEMO_PITCH_DEG,0.0)#用示例参数造外参
    projector = GroundProjector(DEMO_K,DEMO_DIST,R,t,min_range_m=0.3,max_range_m=15.0)
    print(f"  示例相机：{int(DEMO_K[0, 2] * 2)}x{int(DEMO_K[1, 2] * 2)}  fx={DEMO_K[0, 0]:.0f}  "
          f"高度 {DEMO_HEIGHT_M} m  俯角 {DEMO_PITCH_DEG}°")

    #---- 1) 几何往返：不加噪声应几乎无误差 ---- #
    worst = 0.0#记录最大误差
    checked = 0
    for f in field_map.features:#每个地图点都试一遍
        #换三个微小的相机位置，避免碰巧对齐的假通过。
        for dx,dy in ((0.0,0.0),(0.12,-0.07),(-0.2,0.15)):
            pose = Pose2D(f.x - 1.8 + dx,f.y - 0.3 + dy,0.0)#相机放在特征点旁边
            inside,uv,_ = _project_map_point(pose,(f.x,f.y),DEMO_K,DEMO_DIST,R,t)
            if not inside:#不在画面里就跳过（不算错误）
                continue
            gp = projector.project_pixel(float(uv[0]),float(uv[1]))#像素再投回地面
            if gp is None:#又出了有效范围，跳过
                continue
            checked += 1
            back = pose.apply([[gp.x,gp.y]])[0]#变回地图坐标
            worst = max(worst,float(math.hypot(back[0] - f.x,back[1] - f.y)))#记误差
    ok_roundtrip = checked>0 and worst < 1e-6#1e-6 米 = 1 微米，纯粹是浮点误差的量级
    #三元表达式：通过就打印 OK，否则 FAIL。
    print(f"  [1] 像素<->地面往返 {checked} 点，最大误差 {worst:.2e} m  -> {'OK' if ok_roundtrip else 'FAIL'}")

    #---- 2) 位姿估计：加像素噪声后应能收敛到厘米级 ---- #
    rng = np.random.default_rng(seed)#固定种子的随机数生成器，保证结果可复现
    x_min,x_max,y_min,y_max = field_map.bounds()
    errs,yaw_errs = [],[]#收集位置误差和角度误差
    valid_frames = 0#有效帧数
    tried = 0#尝试次数
    status_count: dict = {}#各种状态出现多少次
    while tried < trials and len(errs) < trials:#两个条件都要满足才继续
        tried += 1
        gt = Pose2D(#gt = ground truth，真值
            float(rng.uniform(x_min + 0.5,x_max - 0.5)),#uniform 在区间内均匀取一个数
            float(rng.uniform(y_min + 0.5,y_max - 0.5)),
            float(rng.uniform(-180.0,180.0)),
        )
        obs = []#这一帧的观测
        for f in field_map.features:
            inside,uv,depth = _project_map_point(gt,(f.x,f.y),DEMO_K,DEMO_DIST,R,t)
            if not inside or depth > 12.0:#看不到或太远，不参与
                continue
            #给像素加一点正态分布噪声，模拟真实检测的抖动。
            u = float(uv[0]) + float(rng.normal(0.0,noise_px))
            v = float(uv[1]) + float(rng.normal(0.0,noise_px))
            gp = projector.project_pixel(u,v)#噪声后的像素投回地面
            if gp is None:
                continue
            obs.append(Observation(type=f.type,x=gp.x,y=gp.y,conf=0.9))
        if len(obs) < 2:#少于两个点解不出位姿
            continue
        valid_frames += 1
        match = estimate_pose(obs,field_map,min_markers=2,max_markers=8,max_residual_m=0.45)
        #字典计数：没有这个状态就先给 0，再加 1。
        status_count[match.status] = status_count.get(match.status,0) + 1
        if match.valid and match.pose is not None:
            errs.append(math.hypot(match.pose.x - gt.x,match.pose.y - gt.y))
            yaw_errs.append(abs(wrap_deg(match.pose.yaw_deg - gt.yaw_deg)))#abs 取绝对值

    if errs:
        e = np.array(errs)#转成 numpy 数组才能用 mean/median
        y = np.array(yaw_errs)
        print(
            f"  [2] 位姿估计 {len(errs)}/{valid_frames} 帧有效（像素噪声 {noise_px:.1f} px）\n"
            #*100 是把米换成厘米；np.median 是中位数（比平均值更抗异常值）。
            f"      位置误差 均值 {e.mean() * 100:.1f} cm / 中位 {np.median(e) * 100:.1f} cm / 最大 {e.max() * 100:.1f} cm\n"
            f"      航向误差 均值 {y.mean():.2f}° / 最大 {y.max():.2f}°"
        )
        ok_pose = e.mean() < 0.15 and y.mean() < 3.0#门槛：平均 15 厘米、平均 3 度
    else:
        print("  [2] 没有任何一帧能出结果 -> FAIL（几何或匹配有问题）")
        ok_pose = False
    print(f"      状态分布 {status_count}")

    #---- 3) 失败路径：只 1 个观测应报 insufficient_markers ---- #
    one = [Observation(type="LCross",x=2.0,y=-1.0,conf=0.9)]#只有一条观测
    bad = estimate_pose(one,field_map)
    #两个条件都要满足（and）：结果必须无效，且状态必须是标志不足。
    ok_guard = (not bad.valid) and bad.status == "insufficient_markers"
    print(f"  [3] 只 1 个标志 -> status={bad.status} valid={bad.valid}  "
          f"-> {'OK' if ok_guard else 'FAIL'}")

    all_ok = ok_roundtrip and ok_pose and ok_guard#三项全过才算过
    print("=" * 66)
    print("自检结论：" + ("全部通过，可以开始放视频数据" if all_ok else "有项目未通过，先看上面哪一项 FAIL"))
    return all_ok


#--------------------------------------------------------------------------- #
#工程配置装载
#--------------------------------------------------------------------------- #
def load_project(root: Path,config_rel: str = "config/camera.yaml"):
    #读运行参数、内参、外参和场地地图
    cfg = load_runtime_config(str(root / config_rel))#"/" 在 Path 里表示拼接
    intr = load_intrinsics(str(root / "calibration/camera_intrinsics.yaml"))
    extr = load_extrinsics(str(root / "calibration/camera_extrinsics.yaml"))
    field_map = FieldMap.load(str(root / "config/field.yaml"))
    #一次返回四样东西，调用处用元组解包接收。
    return cfg,intr,extr,field_map


def resolve_weights(path_str: str,root: Path) -> str:
    #相对权重路径从项目根目录算起，路径为空就直接返回
    if not path_str:#空字符串 -> 没有权重，直接返回
        return ""
    p = Path(path_str)
    #三元表达式：绝对路径就原样用，相对路径就拼到项目根上。
    return str(p if p.is_absolute() else (root / p))


#--------------------------------------------------------------------------- #
#主循环
#--------------------------------------------------------------------------- #
def run_live(args) -> int:
    #逐帧做定位，正常结束返回0，出错返回错误码
    root = Path(args.root).resolve()#resolve() 把相对路径变成绝对路径
    cfg,intr,extr,field_map = load_project(root,args.config)
    print(f"[config] 项目根 {root}")
    #三元表达式写在 f-string 里，输出已标定 / 未标定。
    print(f"[config] 相机 {intr.width}x{intr.height}  内参{'已标定' if intr.is_calibrated else '未标定'}  "
          f"外参{'已标定' if extr.is_calibrated else '未标定'}")

    if not intr.is_calibrated or not extr.is_calibrated:#有一项没标定
        if not args.allow_uncalibrated:#又没加 --allow-uncalibrated
            print("[error] 内参/外参还是 TODO 占位。先跑：")
            print("         python src/soccer_vision/calibrate.py --images videos/raw/calib/*.png --out calibration/camera_intrinsics.yaml")
            print("         python src/soccer_vision/extrinsics.py --height 0.42 --pitch 18 --out calibration/camera_extrinsics.yaml")
            print("         或者先用 --allow-uncalibrated 看界面能不能起来（结果不可信）。")
            return 2#提前返回错误码
        print("[warn] 使用示例内参/外参，只为了能看到界面，定位结果不可信")
        K,dist = DEMO_K,DEMO_DIST#多变量同时赋值
        R,t = build_extrinsics(DEMO_HEIGHT_M,DEMO_PITCH_DEG,0.0)
    else:
        K,dist,R,t = intr.K,intr.dist,extr.R,extr.t#用真正标定出来的参数

    geo = cfg["geometry"]#字典取子项
    projector = GroundProjector(K,dist,R,t,geo["min_range_m"],geo["max_range_m"])

    #-- 检测器（缺 ultralytics / 缺权重时只是 unavailable，不会崩） -- #
    det_cfg = cfg["detect"]
    ball_det = YoloDetector(#足球检测器
        resolve_weights(det_cfg["ball_weights"],root),
        conf=float(det_cfg["ball_conf"]),
        imgsz=int(det_cfg["imgsz"]),
        device=str(det_cfg["device"]),
        expected_names=("ball",),#只认 ball 这一类
    )
    marker_det = YoloDetector(#场地标志检测器
        resolve_weights(det_cfg["marker_weights"],root),
        conf=float(det_cfg["marker_conf"]),
        imgsz=int(det_cfg["imgsz"]),
        device=str(det_cfg["device"]),
        expected_names=("LCross","TCross","XCross","PenaltyPoint"),
    )
    print(f"[detect] 足球  {ball_det.describe()}")
    print(f"[detect] 标志  {marker_det.describe()}")

    localizer = Localizer(projector,field_map,cfg["match"],cfg["filter"])#串起定位链路
    canvas = MapCanvas(field_map)#右边地图画布
    #有 jsonl 路径就写文件，否则只打印。
    log_path = cfg["output"]["jsonl"]

    #-- 视频源 -- #
    #args.source 为空就用配置里的相机编号；`or` 的短路特性正好用来做默认值。
    source_str = args.source or str(cfg["camera"]["device"])
    src = FrameSource(
        source_str,
        width=int(cfg["camera"]["width"]),
        height=int(cfg["camera"]["height"]),
        fps=int(cfg["camera"]["fps"]),
    )
    if not src.open():#打不开就直接退出
        print(f"[error] 打不开视频源：{source_str}")
        return 3

    publisher = make_publisher(root/log_path if log_path else None)
    writer = None
    show = not args.no_display#是否开窗口
    win = "soccer_vision  (q/ESC 退出)"#窗口标题
    fps = 0.0#平滑后的帧率
    frames = 0#处理了多少帧
    loc_ok = 0#定位有效的帧数
    status_hist: dict = {}#状态统计
    marker_sum = 0#标志总数累加（最后算平均）
    t_start = time.perf_counter()#开始时间（perf_counter 是最高精度计时器）
    last_log = t_start#上次打印日志的时间

    try:#try/finally：不管怎么退出，finally 都会执行
        while True:#一直循环，直到读到帧失败或用户退出
            ok,frame = src.read()
            if not ok:#视频读完了
                break
            t0 = time.perf_counter()#记下这一帧开始的时间

            ball_dets = ball_det.infer(frame)#检测足球
            marker_dets = marker_det.infer(frame)#检测场地标志
            infer_ms = (time.perf_counter() - t0) * 1000.0#秒转毫秒

            out = localizer.update(frame,ball_dets,marker_dets)#定位主链路
            if not (intr.is_calibrated and extr.is_calibrated):
                out.match.valid = False
                out.match.status = "uncalibrated"
            elif (frame.shape[1],frame.shape[0]) != (intr.width,intr.height):
                out.match.valid = False
                out.match.status = "resolution_mismatch"
            #打包成 ROS2 风格的消息对象（没有 ROS2 时走自带的 stub）。
            pose_msg = FieldPose.from_match(out.match,out.pose,extr if extr.is_calibrated else None,frames)
            publisher.publish(pose_msg,out.balls)#发布 / 打印 / 写 jsonl

            st = out.match.status if out.match else "no_match"
            status_hist[st] = status_hist.get(st,0) + 1#状态计数
            if pose_msg.valid:
                loc_ok += 1
            marker_sum += len(out.markers)

            frames += 1
            elapsed = max(time.perf_counter()-t_start,1e-9)
            fps = frames/elapsed
            loc_hz = loc_ok/elapsed#每秒得到几个有效位姿
            camera_pose = None
            if pose_msg.valid:
                camera_pose = Pose2D(pose_msg.x_camera_map,pose_msg.y_camera_map,pose_msg.yaw_camera_map)

            if show or args.save_video:
                left = draw_left(frame,out.balls,out.markers,fps,infer_ms,out.match)
                right = canvas.draw(camera_pose,out.match,fps,loc_hz)
                view = compose(left,right)
            if args.save_video:
                if writer is None:
                    Path(args.save_video).parent.mkdir(parents=True,exist_ok=True)
                    size = (view.shape[1],view.shape[0])
                    writer = cv2.VideoWriter(args.save_video,cv2.VideoWriter_fourcc(*"mp4v"),
                                             max(src.fps_measured,5.0),size)
                    if not writer.isOpened():
                        raise OSError(f"无法写入录像：{args.save_video}")
                writer.write(view)
            if show:
                try:
                    cv2.imshow(win,view)#显示窗口
                    #waitKey(1) 等 1 毫秒并返回按键；& 0xFF 只保留低 8 位（兼容不同平台）。
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27,ord("q")):#27 是 ESC 键的编码；ord("q") 是字母 q
                        break#退出循环
                except Exception as exc:  # noqa: BLE001
                    #没有图形界面（比如在服务器上）时不要崩，改成只打印。
                    print(f"[warn] 无图形界面（{exc.__class__.__name__}），改为只打印日志")
                    show = False

            now = time.perf_counter()
            if now - last_log > 2.0:#每 2 秒打印一次，不刷屏
                last_log = now
                msg = pose_msg.to_dict()#消息转字典，方便取字段
                print(
                    f"[run] frame {frames}  FPS {fps:5.1f}  infer {infer_ms:5.1f} ms  "
                    f"balls {len(out.balls)}  markers {len(out.markers)}({out.dropped} 投影失败)  "
                    f"pose=({msg['x_camera_map']}, {msg['y_camera_map']}, {msg['yaw_camera_map']})  "
                    f"status={out.match.status if out.match else '-'}"
                )

            if args.max_frames and frames >= args.max_frames:#达到帧数上限
                break
    except KeyboardInterrupt:#用户按了 Ctrl+C
        print("\n[run] Ctrl+C 退出")
    finally:#无论如何都要收尾
        src.release()#关视频
        publisher.close()#关日志文件
        if writer is not None:
            writer.release()#关录像（不关会写出损坏的 mp4）
        if show:
            try:
                cv2.destroyAllWindows()#关窗口
            except Exception:  # noqa: BLE001
                pass#没有窗口可关也无所谓

    dur = time.perf_counter() - t_start
    #平均帧率；dur 为 0 时用 `if dur > 0 else 0` 防止除零。
    print(f"[done] {frames} 帧，{dur:.1f} s，平均 {frames / dur if dur > 0 else 0:.1f} FPS")
    if frames:
        print(f"[done] 定位有效 {loc_ok}/{frames} ({loc_ok / frames * 100:.1f}%)  "
              f"平均标志数 {marker_sum / frames:.2f}")
        print(f"[done] 状态分布 {status_hist}")
        if status_hist.get("ok",0) == 0:#一帧都没成功
            print("[hint] 全是无效帧：先看检测器是不是不可用（缺权重/ultralytics），"
                  "再看标定与 field.yaml 是否匹配")
    if cfg["output"]["jsonl"]:
        print(f"[done] 位姿已写入 {root / cfg['output']['jsonl']}")
    return 0



#--------------------------------------------------------------------------- #
#内置合成演示（--demo）：不需要相机、不需要权重、不需要实拍素材
#--------------------------------------------------------------------------- #
def _draw_ground_line(frame,pose,a,b,K,dist,R,t,thickness=2) -> None:
    #把地图上a到b的线段投到画面上画出来
    pts = []#收集投影后的像素点
    for i in range(21):#一条线段采样 21 个点，弯的也能画
        s = i / 20.0#0.0 ~ 1.0 的比例
        mx = a[0] + (b[0] - a[0]) * s#线性插值求 x
        my = a[1] + (b[1] - a[1]) * s#线性插值求 y
        inside,uv,depth = _project_map_point(pose,(mx,my),K,dist,R,t)
        if not inside or depth <= 0.05:#出画 / 在相机背后就丢掉这个点
            continue
        pts.append([int(round(uv[0])),int(round(uv[1]))])
    if len(pts) >= 2:#至少两个点才能连成线
        cv2.polylines(frame,[np.array(pts,dtype=np.int32)],False,(235,235,235),thickness,cv2.LINE_AA)


def _draw_marker(frame,uv,half_px,name) -> None:
    #按类型画标志，把交点或圆心放在像素点uv上
    u,v = int(round(uv[0])),int(round(uv[1]))#像素坐标取整
    s = max(6,int(half_px))#半边长度，最小 6 像素免得看不见
    ink = (35,35,35)#标志用近黑色画
    if name == "TCross":#T 形：横线过交点，竖线往下
        cv2.line(frame,(u - s,v),(u + s,v),ink,3,cv2.LINE_AA)
        cv2.line(frame,(u,v),(u,v + s),ink,3,cv2.LINE_AA)
    elif name == "LCross":#L 形：右和下两条，交点在左上
        cv2.line(frame,(u,v),(u + s,v),ink,3,cv2.LINE_AA)
        cv2.line(frame,(u,v),(u,v + s),ink,3,cv2.LINE_AA)
    elif name == "XCross":#X 形：两条对角线交于中心
        cv2.line(frame,(u - s,v - s),(u + s,v + s),ink,3,cv2.LINE_AA)
        cv2.line(frame,(u - s,v + s),(u + s,v - s),ink,3,cv2.LINE_AA)
    else:#PenaltyPoint：实心圆点
        cv2.circle(frame,(u,v),max(3,s // 2),(140,140,140),-1,cv2.LINE_AA)


def _render_demo_frame(field_map,pose,K,dist,R,t,ball_xy,rng):
    #画一帧模拟画面，同时返回这一帧的球框和标志框
    w,h = int(K[0,2] * 2),int(K[1,2] * 2)#图像尺寸由主点推出来
    base = np.full((h,w,3),88,dtype=np.uint8)#灰色地面底色
    noise = rng.normal(0.0,4.0,base.shape)#一点随机噪声，看着像真图
    frame = np.clip(base.astype(np.float32) + noise,0,255).astype(np.uint8)

    for y in range(-4,5):#沿 y 方向每米一条横线
        _draw_ground_line(frame,pose,(-1.0,y),(7.0,y),K,dist,R,t)
    for x in range(0,7):#沿 x 方向每米一条竖线
        _draw_ground_line(frame,pose,(x,-4.0),(x,4.0),K,dist,R,t)

    fx = float(K[0,0])#焦距，用来把米换成像素
    markers,balls = [],[]#收集这一帧的检测框

    for idx,feat in enumerate(field_map.features):#遍历地图上 14 个特征点
        inside,uv,depth = _project_map_point(pose,(feat.x,feat.y),K,dist,R,t)
        if not inside or depth > 12.0:#看不到或太远就不画
            continue
        half = 0.16 * fx / max(depth,0.1)#0.16 米见方换算成像素
        _draw_marker(frame,uv,half,feat.type)
        markers.append(Detection(#造一个假的检测结果
            cls_id=idx % 4,name=feat.type,conf=0.93,
            x1=float(uv[0] - half),y1=float(uv[1] - half),
            x2=float(uv[0] + half),y2=float(uv[1] + half),
        ))

    inside,uv,depth = _project_map_point(pose,ball_xy,K,dist,R,t)
    if inside and depth > 0.05:#球在画面里就画出来
        radius = max(4,int(0.11 * fx / max(depth,0.1)))
        cv2.circle(frame,(int(uv[0]),int(uv[1])),radius,(245,245,245),-1,cv2.LINE_AA)
        for k in range(5):#足球表面随手点几个黑点
            ang = k * 2 * math.pi / 5 + 0.4
            px = int(uv[0] + radius * 0.55 * math.cos(ang))
            py = int(uv[1] + radius * 0.55 * math.sin(ang))
            cv2.circle(frame,(px,py),max(1,radius // 5),(25,25,25),-1,cv2.LINE_AA)
        balls.append(Detection(
            cls_id=0,name="ball",conf=0.95,
            x1=float(uv[0] - radius),y1=float(uv[1] - radius),
            x2=float(uv[0] + radius),y2=float(uv[1] + radius),
        ))
    return frame,balls,markers


def run_demo_mode(args) -> int:
    #用程序画的模拟画面跑一遍定位流程
    root = Path(args.root).resolve()
    cfg,_,_,field_map = load_project(root,args.config)
    K,dist = DEMO_K,DEMO_DIST#示例内参，不读标定文件
    R,t = build_extrinsics(DEMO_HEIGHT_M,DEMO_PITCH_DEG,0.0)#示例外参

    geo = cfg["geometry"]
    projector = GroundProjector(K,dist,R,t,geo["min_range_m"],geo["max_range_m"])
    localizer = Localizer(projector,field_map,cfg["match"],cfg["filter"])
    canvas = MapCanvas(field_map)
    publisher = make_publisher(None,prefer_ros2=False)#demo 不写 jsonl

    print("[demo] 内置合成场景：不用相机、不用权重，整条链路照跑")
    print(f"[demo] 地图 {field_map.source}  14 点 {field_map.counts()}")
    print(f"[demo] 示例相机 {int(K[0, 2] * 2)}x{int(K[1, 2] * 2)}  fx={K[0, 0]:.0f}  "
          f"高 {DEMO_HEIGHT_M} m  俯角 {DEMO_PITCH_DEG}°")

    writer = None#录像器，默认不录
    if args.save_video:#传了 --save-video 才建
        os.makedirs(os.path.dirname(os.path.abspath(args.save_video)) or ".",exist_ok=True)
        vw = int(K[0,2] * 2)#左画面宽（1280）
        vh = int(K[1,2] * 2)#左画面高（720）
        writer = cv2.VideoWriter(#拼好的宽度 = 左 + 4 px 分隔条 + 右
            args.save_video,cv2.VideoWriter_fourcc(*"mp4v"),30.0,
            (vw + canvas.width + 4,max(vh,canvas.height)),
        )
        print(f"[demo] 录像 -> {args.save_video}")

    show = not args.no_display
    win = "soccer_vision  DEMO  (q/ESC 退出)"
    rng = np.random.default_rng(7)#固定种子，噪声可复现
    fps = 0.0
    frames = 0
    loc_ok = 0
    status_hist: dict = {}
    t_start = time.perf_counter()
    last_log = t_start

    try:
        while True:
            t_sec = frames / 30.0#按 30 fps 推进时间轴
            gt = Pose2D(#真值：相机慢慢晃
                3.0 + 0.15 * math.sin(0.25 * t_sec),
                -1.5 + 0.10 * math.cos(0.40 * t_sec),
                90.0 + 8.0 * math.sin(0.50 * t_sec),
            )
            ball_xy = (2.60,-0.60 + 1.30 * math.sin(0.60 * t_sec))
            t0 = time.perf_counter()
            frame,balls,markers = _render_demo_frame(field_map,gt,K,dist,R,t,ball_xy,rng)

            out = localizer.update(frame,balls,markers)
            pose_msg = FieldPose.from_match(out.match,out.pose,None,frames)
            publisher.publish(pose_msg,out.balls)

            st = out.match.status if out.match else "no_match"
            status_hist[st] = status_hist.get(st,0) + 1
            if pose_msg.valid:
                loc_ok += 1
            frames += 1

            elapsed = max(time.perf_counter()-t_start,1e-9)
            fps = frames/elapsed
            loc_hz = loc_ok/elapsed

            left = draw_left(frame,out.balls,out.markers,fps,0.0,out.match)
            right = canvas.draw(out.pose,out.match,fps,loc_hz)
            view = compose(left,right)#左边画面 + 右边地图
            if writer is not None:
                writer.write(view)#要录就写进 mp4

            if show:
                try:
                    cv2.imshow(win,view)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27,ord("q")):
                        break
                except Exception as exc:  # noqa: BLE001
                    print(f"[warn] 无图形界面（{exc.__class__.__name__}），改为只打印日志")
                    show = False

            now = time.perf_counter()
            if now - last_log > 2.0:#每 2 秒打一行
                last_log = now
                msg = pose_msg.to_dict()
                print(f"[demo] frame {frames}  FPS {fps:5.1f}  balls {len(out.balls)}  "
                      f"markers {len(out.markers)}  status={st}  "
                      f"pose=({msg['x_camera_map']}, {msg['y_camera_map']}, {msg['yaw_camera_map']})")

            if args.max_frames and frames >= args.max_frames:
                break
            time.sleep(max(0.0,1.0 / 30.0 - (time.perf_counter() - t0)))#限速到 30 fps
    except KeyboardInterrupt:
        print("\n[demo] Ctrl+C 退出")
    finally:
        publisher.close()
        if writer is not None:
            writer.release()#不关会写出损坏的 mp4
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass

    dur = time.perf_counter() - t_start
    print(f"[demo] {frames} 帧，{dur:.1f} s，平均 {frames / dur if dur > 0 else 0:.1f} FPS")
    print(f"[demo] 定位有效 {loc_ok}/{frames} ({loc_ok / frames * 100 if frames else 0:.1f}%)  "
          f"状态分布 {status_hist}")
    return 0


#--------------------------------------------------------------------------- #
def build_argparser() -> argparse.ArgumentParser:
    #设置命令行参数，加--help可以看用法
    ap = argparse.ArgumentParser(description="足球检测 + 单目场地定位演示")
    #default= 是默认值；help= 是帮助文字。
    ap.add_argument("--root",default=str(Path(__file__).resolve().parents[2]),help="项目根目录")
    ap.add_argument("--config",default="config/camera.yaml",help="运行时配置文件（相对项目根）")
    ap.add_argument("--source",default="",help="视频文件 / 视频目录 / 相机编号 0；留空用配置里的 device")
    #action="store_true"：不加参数值时默认 False，写了这个参数就变 True（开关）。
    ap.add_argument("--check",action="store_true",help="只跑合成数据自检，不需要视频与权重")
    ap.add_argument("--demo",action="store_true",help="内置合成演示：不用相机/权重/标定，直接看左右画面与箭头")
    ap.add_argument("--no-display",action="store_true",help="不开窗口（无桌面/录屏时用）")
    ap.add_argument("--save-video",default="",help="把左右拼接画面录成 mp4")
    ap.add_argument("--max-frames",type=int,default=0,help="最多处理多少帧，0 表示不限制")
    ap.add_argument("--allow-uncalibrated",action="store_true",help="内参外参未标定也强行运行（结果不可信）")
    return ap


def main(argv=None) -> int:
    #先读命令行参数，再按选项启动对应模式
    args = build_argparser().parse_args(argv)#不传 argv 就自动用 sys.argv（真实命令行）
    root = Path(args.root).resolve()

    if args.demo:#内置合成演示
        return run_demo_mode(args)#走演示分支
    if args.check:#只自检
        _,_,_,field_map = load_project(root,args.config)#只用第四项，前三个用 _ 忽略
        ok = synthetic_self_test(field_map)
        return 0 if ok else 1#成功返回 0，失败返回 1
    return run_live(args)#否则跑实时循环


#只有直接运行这个文件时，下面的代码才执行；被别的文件 import 时不执行。
if __name__ == "__main__":
    raise SystemExit(main())#把 main() 的返回值当进程退出码
