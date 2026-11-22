"""Launch log adapters without opening any capture device."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def nodes(context):
    here=Path(__file__).resolve().parent
    python=here.parents[1]/".venv/bin/python"
    mode=LaunchConfiguration("mode").perform(context)
    if mode not in ("pose","ball","both"):
        raise ValueError("mode must be pose, ball, or both")
    ret=[]
    for kind in ("ball","pose"):
        if mode not in (kind,"both"):
            continue
        path=LaunchConfiguration(kind+"_log").perform(context)
        if not path:
            raise ValueError("supply "+kind+"_log")
        follow=LaunchConfiguration("follow").perform(context)
        ret.append(ExecuteProcess(cmd=[str(python),"-B",str(here/"log_node.py"),kind,
                     "--ros-args","-p","log_path:="+path,"-p","follow:="+follow],
                     name="cquick_"+kind,output="screen"))
    return ret


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("mode",default_value="pose"),
        DeclareLaunchArgument("ball_log",default_value=""),
        DeclareLaunchArgument("pose_log",default_value=""),
        DeclareLaunchArgument("follow",default_value="false"), OpaqueFunction(function=nodes)])
