from Motor import Stepper

motor = Stepper("/dev/ttyACM0")

#motor.brake()
#motor.release_brake()
motor.calibrate()
#motor.release_brake
#motor.move_stepper(-1000)

