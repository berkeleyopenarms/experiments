from time import time

class PID:
    def __init__(self, Kp, Ki, Kd, Ff=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ff = (Ff) if callable(Ff) else (lambda n: Ff * n)

        self.setpoint = None

        self.error_prev = None
        self.error_sum = 0.0
        self.time_prev = 0.0

    def calc(self, state):
        if self.setpoint == None:
            return 0

        time_curr = time()

        error = self.setpoint - state

        effort = self.Kp * error + self.Ki * self.error_sum + self.Ff(self.setpoint)
        if self.error_prev != None and time_curr > self.time_prev:
            dt = time_curr - self.time_prev
            effort += (error - self.error_prev) * self.Kd / dt
            self.error_sum += (error + self.error_prev) / 2 * dt

        self.error_prev = error
        self.time_prev = time_curr

        return effort

    def clear_integrator(self):
        self.error_sum = 0

if __name__ == '__main__':
    from time import sleep
    p = PID(0.2, 1, 0)
    p.setpoint = 20
    x = 10
    while True:
        print int(x) * '.' + int(50 - x) * 'x'
        x += p.calc(x)
        sleep(0.1)
