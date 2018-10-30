from led_pwm_proxy import LedPwmProxy

class LedScheduler(object):

    def __init__(self) :

        self.led_pin = 3
        self.led_on_value = 255
        self.led_off_value = 0

        self.on_duration = 1.0
        self.minimum_off_duration = 9.0

        self.led_on = False 
        self.last_on_t  = 0.0
        self.activation_count = 0

        self.led_proxy= LedPwmProxy()
        self.turn_off_led()

    def __del__(self):
        self.turn_off_led()

    def turn_on_led(self):
        self.led_proxy.set_value(self.led_pin, self.led_on_value)
        self.activation_count += 1

    def turn_off_led(self):
        self.led_proxy.set_value(self.led_pin, self.led_off_value)

    def update(self, t, fly_on_food):
        if self.led_on:
            if (t - self.last_on_t) > self.on_duration:
                self.led_on = False
                self.turn_off_led()
        else:
            if fly_on_food:
                if (t - self.last_on_t) > (self.on_duration + self.minimum_off_duration):
                    self.led_on = True 
                    self.last_on_t = t
                    self.turn_on_led()

