#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from shared_objects.Motor_old import Stepper
from shared_objects.ROS_utils import Topics
from datetime import datetime
from pathlib import Path

# Parametri di conversione e limiti
TRANSMISSION_RATIO = 14
MAX_ANGLE = 10
STEP_ANGLE = 1.8  # in gradi

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, target, actual, dt):
        error = target - actual
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        # Inizializza lo stepper con i parametri necessari
        self.steering = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,  # modulo dello sterzo
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,  # parametro di esempio: regola se necessario
            V1=100_000,
            A1=50_000,             # parametro di esempio: regola se necessario
            D1=50_000              # parametro di esempio: regola se necessario
        )

        # Inizializza il PID (tuning dei parametri da effettuare in base al sistema)
        self.pid = PIDController(kp=1.0, ki=0.0, kd=0.0)

        # Variabile per memorizzare il target (in gradi) ricevuto via ROS
        self.latest_angle = 0.0

        self.declare_parameter(
            'debug_root',
            '/home/bylogix/Shell-Eco-Marathon-2025/DEBUG'  # default path
        )
        self.debug_root = Path(
            self.get_parameter('debug_root').get_parameter_value().string_value
        )
        self.debug_root.mkdir(parents=True, exist_ok=True)

        # Create a timestamped log file for PID logs
        #timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        #counter =0
        #counter+=1
        #self.log_file = self.debug_root / timestamp / f"pid_log_{counter}.txt"

        #self.get_logger().info(f"PID logs is in -> {self.log_file}")


        # QoS per il subscriber
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        topics = Topics()
        topic_names = topics.topic_names

        self.subscription = self.create_subscription(
            Float64,
            topic_names["steering"],
            self.callback_stepangle,
            qos_profile
        )

        # Timer a 100 Hz per il controllo PID (dt = 0.01 s)
        self.pid_timer = self.create_timer(0.01, self.pid_loop)

        self.get_logger().info("Steering node with PID at 100Hz initialized")

    def callback_stepangle(self, msg):
        """
        Callback che aggiorna il target angle (in gradi) ricevuto.
        """
        self.latest_angle = msg.data
        self.get_logger().info(f"Received angle: {self.latest_angle}")

    def convert_angle_to_position(self, angle):
        """
        Converte l'angolo target (in gradi) in posizione (in microstep) coerente con il motore.
        """
        # Calcola il numero di step: ogni step è STEP_ANGLE gradi, convertito in microstep (256 microstep per step)
        steps = (angle / STEP_ANGLE) * 256
        # Applica il rapporto di trasmissione; il segno meno può essere necessario per la corretta direzione
        return -int(steps * TRANSMISSION_RATIO)

    def pid_loop(self):
        """
        Loop eseguito a 100 Hz per aggiornare il controllo PID.
        """
        dt = 0.01  # intervallo del timer in secondi

        # Aggiorna la posizione "unwrapped" del motore
        current_position = self.steering.update_unwrapped_position()

        # Calcola la posizione target in microstep a partire dall'angolo ricevuto (con saturazione se necessario)
        if self.latest_angle > MAX_ANGLE:
            target_angle = MAX_ANGLE
        elif self.latest_angle < -MAX_ANGLE:
            target_angle = -MAX_ANGLE
        else:
            target_angle = self.latest_angle

        target_position = self.convert_angle_to_position(target_angle)

        # Calcola l'output del PID
        pid_output = self.pid.compute(target_position, current_position, dt)

        # Calcola il nuovo comando sommando l'output PID alla posizione attuale
        new_command = int(current_position + pid_output)

        self.get_logger().info(
            f"PID loop: target={target_position}, current={current_position}, output={pid_output}, command={new_command}"
        )

        #PID loop information to the log file
        #with open(self.log_file, "a") as log:
        #    log.write(f"PID loop: target={target_position}, current={current_position}, output={pid_output}, command={new_command}\n")

        # Invia il comando al motore
        self.steering.move_stepper(new_command)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Steering node shutting down")
    finally:
        node.steering.disconnect_motor()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
