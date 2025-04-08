#!/usr/bin/env python3

# import Jetson.GPIO as GPIO
import RPi.GPIO as GPIO
import os
import time
import subprocess

INPUT_PIN = 15 # GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(True)
GPIO.setmode(GPIO.BOARD) #BCM or BOARD but if you use BOARD numbering changes
GPIO.setup(INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print(f"Listening the PIN: {INPUT_PIN}")

def enable_launch():
  global ros_process
  try:
      print("Launching ROS 2 nodes...")
      ros_process = subprocess.Popen(
          ["ros2", "launch", "juno_bringup", "path_planning.launch.py"],
          preexec_fn=os.setsid  # Create a new process group // allows proper termination of child process
      )
  except Exception as e:
      print(f"Failed to launch ROS nodes: {e}")


def stop_launch():

    global ros_process
    if subprocess:
      print("Stopping ROS 2 nodes...")
      try:
          # Send SIGTERM to the entire process group
          os.killpg(os.getpgid(ros_process.pid), 15)
          ros_process.wait(timeout=5)
      except subprocess.TimeoutExpired:
          print("Force killing ROS nodes")
          os.killpg(os.getpgid(ros_process.pid), 9)
      except Exception as e:
          print(f"Error stopping ROS nodes: {e}")
      finally:
          ros_process = None

try:
  last_state = GPIO.input(INPUT_PIN)
  print(f"Initial state: {'HIGH (Button pressed)' if last_state else 'LOW (Button released)'}")

  while True:
    current_state = GPIO.input(INPUT_PIN)

    if current_state != last_state:
       if current_state:
          print("Button Pressed")
          enable_launch()
       else:
          print("Button Released")
          stop_launch()
       last_state = current_state
    time.sleep(0.1)
except KeyboardInterrupt:
  print("\nExiting")

finally:
   stop_launch()
   GPIO.cleanup()
