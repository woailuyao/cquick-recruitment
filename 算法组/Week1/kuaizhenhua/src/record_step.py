"""补充 GitHub 要求的真实运行截图：显示姓名、当前时间、命令和实际结果。

从个人项目目录运行 python src/record_step.py 1，按 1～6 分步操作。
本脚本不生成截图、不修改旧图、不模拟终端结果。运行后由本人截图保存。
--check-video 只用于无窗口核验，不表示本人已观看视频。
"""
import argparse
from datetime import datetime, timedelta, timezone
import os
from pathlib import Path
import subprocess
import sys

from common import PROJECT_ROOT, write_json


STEPS = {
    '1': ('环境检查', '01_environment.png'),
    '2': ('任务一', '02_task1.png'),
    '3': ('任务二', '03_task2.png'),
    '4': ('任务三与新测试图', '04_task3.png'),
    '5': ('暗光参数调整', '05_dark_adjustment.png'),
    '6': ('选做视频', '06_video.png'),
}


def timestamp():
    """使用实际运行时间，按中国标准时间显示，不使用写死的历史日期。"""
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')


def identity():
    current = timestamp()
    print(f'姓名: kuaizhenhua | 时间: {current}', flush=True)
    return current


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', choices=STEPS, help='1环境 2任务一 3任务二 4任务三 5暗光调整 6视频')
    parser.add_argument('--check-video', action='store_true', help='只核验视频，不显示窗口')
    args = parser.parse_args()
    if args.check_video and args.step != '6':
        parser.error('--check-video 仅用于第6步。')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    os.chdir(PROJECT_ROOT)
    env = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8')
    title, screenshot = STEPS[args.step]
    started = identity()
    print(f'步骤 {args.step}：{title}', flush=True)
    if args.step == '1':
        commands = [['-c', "import sys,cv2,numpy,matplotlib; print('Python:',sys.version.split()[0]); print('OpenCV:',cv2.__version__); print('NumPy:',numpy.__version__); print('Matplotlib:',matplotlib.__version__)"]]
    elif args.step == '2':
        commands = [['src/task1_basics.py']]
    elif args.step == '3':
        commands = [['src/task2_processing.py']]
    elif args.step == '4':
        commands = [
            ['src/task3_color_detect.py'],
            ['src/task3_color_detect.py', '--input-dir', 'data/images/holdout',
             '--output-dir', 'outputs/task3_holdout'],
        ]
    elif args.step == '5':
        commands = [['src/task3_color_detect.py', '--config', 'src/configs/blue_dark.json',
                     '--output-dir', 'outputs/task3/dark_adjustment']]
    else:
        video = Path('data/videos/demo_color_motion.avi')
        commands = []
        if not video.is_file():
            # Git 中保留了合成素材清单；无 AVI 时重建这份合成视频及清单。
            commands.append(['src/make_demo_video.py', '--force'])
        command = ['src/task4_camera.py', '--video', video.as_posix()]
        if args.check_video:
            command.append('--no-display')
        commands.append(command)
    records = []
    error = None
    try:
        for command in commands:
            # print 只展示随后真正执行的同一条命令；继承子进程输出，不伪造结果。
            print('执行：python ' + subprocess.list2cmdline(command), flush=True)
            result = subprocess.run([sys.executable, *command], cwd=PROJECT_ROOT, env=env)
            records.append({'command': ['python', *command], 'exit_code': result.returncode})
            if result.returncode:
                raise RuntimeError(f'运行失败，退出码 {result.returncode}。先解决报错，不要当作通过截图。')
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or '运行被中断'
    finally:
        finished = identity()
        # 当前时间与结果放在同一屏，便于按要求截取；旧截图不应后期加时间冒充。
        print('结果：' + ('未完成：' + error if error else '本步骤运行成功。'), flush=True)
        if not error:
            print(f'截图保存到：outputs/records/{screenshot}', flush=True)
            print('截图需包含上面的姓名、时间、执行命令与运行结果。', flush=True)
        folder = PROJECT_ROOT / 'outputs/records'
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
        write_json(folder / 'runs' / f'step{args.step}_{stamp}.json', {
            'name_pinyin': 'kuaizhenhua', 'step': args.step, 'started_at': started,
            'finished_at': finished, 'commands': records, 'error': error,
            'check_video_without_display': args.check_video,
            'screenshot_created_by_script': False,
            'note': '这是实际命令执行记录；不证明操作者身份，不替代本人查看与真实截图。',
        })
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
