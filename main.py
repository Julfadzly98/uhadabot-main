from uhadabot.uroslibpy import Ros, Topic, Message
from boot import CONFIG
import machine
from machine import Pin, PWM, Timer
import math
import time
import logging
# from time import sleep

logger = logging.getLogger(__name__)

BUILTIN_LED = 2


M_LEFT_PWM = 12
M_RIGHT_PWM = 14  # nope
M_LEFT_FR = 13
M_RIGHT_FR = 15  # nope

EN_LEFT = 16
EN_RIGHT = 17


__version__ = '0.2.0'
__author__ = 'Roberto Sánchez'
__license__ = "Apache License 2.0. https://www.apache.org/licenses/LICENSE-2.0"


frequency = 15000
pin1 = Pin(12, Pin.OUT)
pin2 = Pin(13, Pin.OUT)
pin3 = Pin(14, Pin.OUT)
pin4 = Pin(15, Pin.OUT)

enable = PWM(Pin(13), frequency)
dc_motor = (pin1, pin2, enable)
dc_motor = (pin1, pin2, enable, 350, 1023)
dc_motor = (pin3, pin4, enable)
dc_motor = (pin3, pin4, enable, 350, 1023)


class DCMotor:
    def __init__(
            self, pin1, pin2, pin3, pin4, enable_pin, min_duty=750,
            max_duty=1023):
        self.pin1 = pin1
        self.pin2 = pin2
        self.pin3 = pin3
        self.pin4 = pin4
        self.enable_pin = enable_pin
        self.min_duty = min_duty
        self.max_duty = max_duty

    def forwards(speed):
        speed = speed
        # self.enable_pin.duty(self.duty_cycle(speed))
        pin1.value(0)
        pin2.value(1)
        pin3.value(0)
        pin4.value(1)

    def backwards(speed):
        speed = speed
        # self.enable_pin.duty(self.duty_cycle(self.speed))
        pin1.value(1)
        pin2.value(0)
        pin3.value(1)
        pin4.value(0)

    def stop(speed):
        speed = speed
        # self.enable_pin.duty(0)
        pin1.value(0)
        pin2.value(0)
        pin3.value(0)
        pin4.value(0)

    def duty_cycle(self, speed):
        if self.speed <= 0 or self.speed > 100:
            duty_cycle = 0
        else:
            duty_cycle = int(
                self.min_duty + (self.max_duty - self.min_duty) *
                ((self.speed - 1) / (100 - 1)))
            return duty_cycle


class HCSR04:
    """
    Driver to use the untrasonic sensor HC-SR04.
    The sensor range is between 2cm and 4m.
    The timeouts received listening to echo pin are converted to OSError('Out of range')
    """
    # echo_timeout_us is based in chip range limit (400cm)

    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=500*2*30):

        self.echo_timeout_us = echo_timeout_us
        # Init trigger pin (out)
        self.trigger = Pin(trigger_pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)

        # Init echo pin (in)
        self.echo = Pin(echo_pin, mode=Pin.IN, pull=None)

    def _send_pulse_and_wait(self):
        """
        Send the pulse to trigger and listen on echo pin.
        We use the method `machine.time_pulse_us()` to get the microseconds until the echo is received.
        """
        self.trigger.value(0)  # Stabilize the sensor
        time.sleep_us(5)
        self.trigger.value(1)
        # Send a 10us pulse.
        time.sleep_us(10)
        self.trigger.value(0)
        try:
            pulse_time = machine.time_pulse_us(
                self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110:  # 110 = ETIMEDOUT
                raise OSError('Out of range')
            raise ex

    def distance_cm(self):
        """
        Get the distance in centimeters with floating point operations.
        It returns a float
        """
        pulse_time = self._send_pulse_and_wait()

        # To calculate the distance we get the pulse_time and divide it by 2
        # (the pulse walk the distance twice) and by 29.1 becasue
        # the sound speed on air (343.2 m/s), that It's equivalent to
        # 0.034320 cm/us that is 1cm each 29.1us
        cms = (pulse_time / 2) / 29.1
        return cms


###############################################################################


class HCSR04READING:

    def __init__(self, ros, name, pin):
        self.name = name

        self.en_pin = pin
        self.count = 0
        self.prev_pin_val = pin.value()

        self.ros = ros
        self.ros_pub = Topic(
            ros, "hadabot/reading_{}".format(self.name),
            "std_msgs/Float32")

    def publish_radps(self, radps):
        sensors = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=10000)
        distance = sensors.distance_cm() < 5
        # logger.info("Encoder publish {}".format(self.name)
        self.ros_pub.publish(Message(distance))

        if distance == 1:
            DCMotor.forwards(50)

        if distance == 0:
            DCMotor.stop(0)


class Controller:
    TICKS_PER_REVOLUTION = 40.0

    ###########################################################################
    def __init__(self, ros):

        # Motor pings
        self.pwm_pin_left = PWM(Pin(M_LEFT_PWM))
        self.pwm_pin_right = PWM(Pin(M_RIGHT_PWM))  # nope

        self.fr_pin_left = Pin(M_LEFT_FR, Pin.OUT)
        self.fr_pin_right = Pin(M_RIGHT_FR, Pin.OUT)  # nope

        self.pwm_pin_left.duty(0)
        self.pwm_pin_right.duty(0)  # nope
        self.fr_pin_left.off()
        self.fr_pin_right.off()  # nope

        # Encoder
        en_pin_left = Pin(EN_LEFT, Pin.IN)
        en_pin_right = Pin(EN_RIGHT, Pin.IN)

        self.en_left = Encoder(ros, "left", en_pin_left)
        self.en_right = Encoder(ros, "right", en_pin_right)

        # Encoder samples
        self.prev_count_left = 0
        self.prev_count_right = 0
        self.prev_ms = time.ticks_ms()

        # Timer to poll encoders
        self.en_poll_timer = Timer(0)
        self.en_poll_timer.init(period=10, mode=Timer.PERIODIC,
                                callback=self.irq_poll_en)

    ###########################################################################
    def irq_poll_en(self, timer):
        for en in [self.en_right, self.en_left]:
            val = en.en_pin.value()
            if (val ^ en.prev_pin_val) == 1:
                en.count += 1
            en.prev_pin_val = val

    ###########################################################################
    def turn_wheel(self, wheel_power_f32, pwm_pin, fr_pin):
        factor = max(min(wheel_power_f32, 1.0), -1.0)

        if False and wheel_power_f32 == 0.0:
            if pwm_pin == self.pwm_pin_right:
                self.en_right.count = 0
            else:
                self.en_left.count = 0

        if factor >= 0:
            # FR pin lo to go forward
            fr_pin.off()
            pwm_pin.duty(int(1023 * factor))
        else:
            # FR pin hi and 'reverse' pwm to go backwards
            fr_pin.on()
            pwm_pin.duty(1023 - int(1023 * (-1*factor)))
    ###########################################################################

    def right_wheel_cb(self, wheel_power):
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_right, self.fr_pin_right)

    ###########################################################################
    def left_wheel_cb(self, wheel_power):
        self.turn_wheel(
            wheel_power["data"], self.pwm_pin_left, self.fr_pin_left)
    ###########################################################################

    def run_once(self):
        state = machine.disable_irq()
        count_left = self.en_left.count
        count_right = self.en_right.count
        machine.enable_irq(state)

        cur_ms = time.ticks_ms()
        time_delta_ms = time.ticks_diff(cur_ms, self.prev_ms)
        per_second = 1000.0 / float(time_delta_ms)

        # Left encoder
        dcount_left = count_left - self.prev_count_left
        radians = 2 * math.pi * (dcount_left / self.TICKS_PER_REVOLUTION)
        radps_left = radians * per_second
        self.en_left.publish_radps(radps_left)

        # Right encoder
        dcount_right = count_right - self.prev_count_right
        radians = 2 * math.pi * (dcount_right / self.TICKS_PER_REVOLUTION)
        radps_right = radians * per_second
        self.en_right.publish_radps(radps_right)

        # Update previous sample
        self.prev_ms = cur_ms
        self.prev_count_left = count_left
        self.prev_count_right = count_right

        if False:
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_left.name, self.en_left.count))
            logger.info(
                "Encoder count {} - {}".format(
                    self.en_right.name, self.en_right.count))
            if count_left >= 120:
                self.left_wheel_cb({"data": 0.01})
            if count_right >= 120:
                self.right_wheel_cb({"data": 0.01})


###############################################################################
def blink_led(ntimes, end_off):
    builtin_led = Pin(BUILTIN_LED, Pin.OUT)

    # Start off
    builtin_led.off()
    time.sleep(0.5)

    for x in range(0, ntimes):
        builtin_led.off()
        time.sleep(0.25)
        builtin_led.on()
        time.sleep(0.5)

    if end_off:
        builtin_led.off()


###############################################################################
def twist_cmd_cb(twist_msg):
    # Blink and leave LED on
    blink_led(1, end_off=False)


###############################################################################
def main(argv):
    ros = None

    while True:
        sensor = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=10000)

        try:
            hadabot_ip_address = CONFIG["network"]["hadabot_ip_address"]
            boot_button = Pin(0, Pin.IN)
            ##builtin_led = Pin(BUILTIN_LED, Pin.OUT)
            distance = sensor.distance_cm()

            ros = Ros(CONFIG["ros2_web_bridge_ip_addr"])

            # Publish out log info
            log_info = Topic(ros, "hadabot/log/info", "std_msgs/String")
            hrc_sensor = Topic(ros, "hadabot/hrc/sensor", "std_msgs/String")

            # Controller
            controller = Controller(ros)

            # Subscribe to twist topics
            # twist_cmd = Topic(ros, "hadabot/cmd_vel", "geometry_msgs/Twist")
            # twist_cmd.subscribe(twist_cmd_cb)

            # Subscribe to wheel turn callbacks
            rw_sub_topic = Topic(
                ros, "hadabot/wheel_power_right", "std_msgs/Float32")
            rw_sub_topic.subscribe(controller.right_wheel_cb)
            lw_sub_topic = Topic(
                ros, "hadabot/wheel_power_left", "std_msgs/Float32")
            lw_sub_topic.subscribe(controller.left_wheel_cb)

            # Loop forever
            last_hb_ms = time.ticks_ms()
            last_controller_ms = time.ticks_ms()
            while (boot_button.value() == 1):

                ros.run_once()

                cur_ms = time.ticks_ms()

                # Run the controller
                if time.ticks_diff(cur_ms, last_controller_ms) > 500:
                    controller.run_once()
                    last_controller_ms = cur_ms

                # Publish heartbeat message
                if time.ticks_diff(cur_ms, last_hb_ms) > 500:
                    # logger.info("Encoder left - {}".format(en_count_left))
                    # logger.info("Encoder right - {}".format(en_count_right))
                    # hadabot_ip_address ---- testing buang dulu nnti ksi masuk balik
                    log_info.publish(
                        Message(
                            "Hadabot heartbeat - IP {}".format(hadabot_ip_address)))
                    last_hb_ms = cur_ms

                if distance > 5:
                    hrc_sensor.publish(
                        Message("Reading {}".format(distance)))

                time.sleep(0.05)

            logger.info("Boot button pushed... exiting")

        except KeyboardInterrupt:
            logger.info("Keyboard control-c hit... stopping")
        # except Exception as ex:
        #   logger.error("Some other exception hit {}".format(str(ex)))
        #  if ros is not None:
        #     ros.terminate()

            # Blink 5 times
            #blink_led(5, end_off=True)

            #raise ex

        # Shutdown
        # ros.terminate()
        # builtin_led.off()


main(None)
