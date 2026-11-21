import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor # 多线程执行器
from rclpy.callback_groups import ReentrantCallbackGroup # 可重入回调组
from cquick_servo_interfaces.action import MoveServo
from rclpy.action import GoalResponse, CancelResponse
from rclpy.node import Node
import serial
import time
from cquick_servo_interfaces.msg import ServoCommand  #?

class ServoDriver(Node):

    def __init__(self):
        super().__init__("servo_driver")

        self.count=0
        
        self.declare_parameter("port","/tmp/ttyFAKE")
        self.declare_parameter("baudrate",115200)
        self.declare_parameter("pwm_min",900)
        self.declare_parameter("pwm_max",2100)
        self.declare_parameter("default_duration_ms",800)
        self.declare_parameter("servo_ids",[1,2,3,4,5,6])
        self.declare_parameter("startup_reset",True)
        # self.declare_parameter("target_pos",1500)
        
        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        startup_reset = self.get_parameter("startup_reset").value
        servo_ids = self.get_parameter("servo_ids").value

        self.ser = serial.Serial(port,baud,timeout=0.2) # 最大等待时0.2   
        time.sleep(0.5)
        
        if startup_reset:
            for sid in servo_ids:
                cmd = f"#{sid:03d}PULR!"
                self.send(cmd)
                time.sleep(0.05)

         # self.timer = self.create_timer(0.5,self.tick)
        self.get_logger().info("node has built")
        
        self.cb_group = ReentrantCallbackGroup() # 实例化这一个类
        self.action_server = ActionServer(
        self,
        MoveServo,
        "move_servo",
        execute_callback = self.execute_callback,
        goal_callback = self.goal_callback,
        cancel_callback = self.cancel_callback,
        handle_accepted_callback = self.handle_accepted_callback,
        callback_group = self.cb_group
        )
        
        self.cmd_pub = self.create_publisher(ServoCommand, "servo/commands", 10)  #?

    def send(self,cmd):
        try:
            self.ser.write(cmd.encode("ascii"))
            self.ser.flush()
            time.sleep(0.05)
            return self.ser.read(64).decode("ascii",errors="ignore")
        except serial.SerialException:
            self.get_logger().warn("something went wrong")
            return""

    def tick(self):
        lo = self.get_parameter("pwm_min").value
        hi = self.get_parameter("pwm_max").value
        ids = self.get_parameter("servo_ids").value
        dur = self.get_parameter("default_duration_ms").value
        sid = ids[self.count % len(ids)]
        if self.count % 2==0:
            target = hi
        else:
            target = lo
        cmd=f"#{sid:03d}P{target:04d}T{dur:04d}!"
        reply = self.send(cmd)
        self.get_logger().info(f"{cmd} -> {reply}")
        self.count+=1
        
    def goal_callback(self,goal_request):
        sid = goal_request.command.id
        target_pwm = goal_request.command.pwm
        lo = self.get_parameter("pwm_min").value
        hi = self.get_parameter("pwm_max").value
        servo_ids = self.get_parameter("servo_ids").value        
        if sid not in servo_ids:
            self.get_logger().warn(f"收到非法舵机ID：{sid}，拒绝这个目标")
            return GoalResponse.REJECT            
        if target_pwm<lo or target_pwm>hi:
            self.get_logger().warn(f"PWM {target_pwm} 超出范围 [{lo},{hi}]，拒绝")
            return GoalResponse.REJECT            
        self.get_logger().info(f"目标校验通过，接受舵机{sid}的运动指令")
        return GoalResponse.ACCEPT # 自动创建一个goal_handle，同意处理后的命令
        
    def cancel_callback(self,goal_handle):
        self.get_logger().info("收到取消请求，同意取消")
        return CancelResponse.ACCEPT
        
    def handle_accepted_callback(self,goal_handle):
        self.get_logger().info("目标已通过，开始执行")
        goal_handle.execute()
         
    def execute_callback(self,goal_handle):
        goal = goal_handle.request
        servo_id = goal.command.id
        target_pwm = goal.command.pwm
        dur_ms = goal.command.duration_ms        
        self.get_logger().info(f"开始执行：舵机{servo_id}，目标PWM={target_pwm}，时长{dur_ms}ms")
        cmd = f"#{servo_id:03d}P{target_pwm:04d}T{dur_ms:04d}!"
        reply = self.send(cmd)        
        #?
        msg = ServoCommand()
        msg.id = goal.command.id
        msg.pwm = goal.command.pwm
        msg.duration_ms = goal.command.duration_ms
        self.cmd_pub.publish(msg)
        #?
        self.get_logger().info(f"{cmd} -> {reply}")
        feedback = MoveServo.Feedback()
        result = MoveServo.Result()
        target_sec = dur_ms/1000.0
        step = 0.2
        t_time = 0.0
        
        read_cmd = f"#{servo_id:03d}PRAD!"
        pos_reply = self.send(read_cmd)
        try:
            start_pwm = int(pos_reply[5:9])
        except:
            start_pwm = 1500
        
        while t_time<target_sec:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("收到取消，停止运动")
                goal_handle.canceled()
                result = MoveServo.Result()
                result.success = False
                result.reason = "运动被中途取消"
                return result
                
            read_cmd = f"#{servo_id:03d}PRAD!"
            pos_reply = self.send(read_cmd)
            try:
                current_pwm = int(pos_reply[5:9]) 
            except:
                current_pwm = 0
            
            feedback.current_pwm = current_pwm
            feedback.progress = t_time / target_sec
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"实时进度：{feedback.progress:.2f}，当前位置：{feedback.current_pwm}")            
            time.sleep(step)
            t_time += step
        
        goal_handle.succeed()
        result.success = True
        result.final_pwm = target_pwm
        result.reason = "运动正常完成"
        self.get_logger().info(f"任务正常结束，最终目标PWM：{result.final_pwm}")
        return result

def main():
    rclpy.init()
    node = ServoDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

