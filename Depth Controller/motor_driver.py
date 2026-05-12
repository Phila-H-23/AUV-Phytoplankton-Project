# ================================================================
# motor_driver.py
# MicroPython -- Raspberry Pi Pico
# BTS7960 43A H-bridge motor driver
#
# Wiring:
#   RPWM -> GP18  (forward PWM  -- hardware PWM channel 0)
#   LPWM -> GP19  (reverse PWM  -- hardware PWM channel 1)
#   R_EN -> GP17  (right enable -- drive HIGH to enable)
#   L_EN -> GP16  (left enable  -- drive HIGH to enable)
#   VCC  -> 5V    (logic supply for driver board)
#   GND  -> GND
#   B+   -> motor power supply positive (your battery/supply)
#   B-   -> motor power supply negative
#   M+   -> motor terminal A
#   M-   -> motor terminal B
#
# BTS7960 control logic:
#   RPWM has duty cycle, LPWM=0 -> forward (piston retracts -> sinks)
#   LPWM has duty cycle, RPWM=0 -> reverse (piston extends  -> rises)
#   Both zero                   -> coast stop
#   Both 100%                   -> active brake
#
# Current limit: BTS7960 rated 43A, externally limited to 3A
# Voltage ceiling: V_SAFE_MAX = I_LIMIT x Ra = 3A x 0.714 = 2.142V
# This means max duty cycle = V_SAFE_MAX / V_SUPPLY x 100 = ~17.9%
# ================================================================
 
from machine import Pin, PWM
from parameters import V_SAFE_MAX, V_SAFE_MIN
 
PIN_RPWM = 18    # GP18 -- forward PWM
PIN_LPWM = 19    # GP19 -- reverse PWM
PIN_R_EN = 17    # GP17 -- right side enable
PIN_L_EN = 16    # GP16 -- left side enable
PWM_FREQ = 20000 # 20kHz -- above audible range, within BTS7960 capability
 
class BTS7960:
    def __init__(self, v_supply=12.0):
        """
        v_supply : float -- actual supply voltage to motor [V]
                   Measure at B+ and B- terminals with a multimeter.
                   Used to scale voltage command to PWM duty cycle.
 
        Note on duty cycle:
            V_SAFE_MAX = 2.142V, v_supply = 12V
            Max duty cycle = 2.142 / 12 x 100 = 17.9%
            This is intentional -- enforces the 3A current limit.
            The BTS7960 can handle far more but the motor and
            external current limit setting constrain it to 3A.
        """
        self.v_supply = v_supply
 
        # Enable pins -- both must be HIGH to enable the H-bridge
        # Drive immediately so motor is ready as soon as object is created
        self.r_en = Pin(PIN_R_EN, Pin.OUT, value=1)
        self.l_en = Pin(PIN_L_EN, Pin.OUT, value=1)
 
        # PWM channels -- duty_u16 range: 0 (0%) to 65535 (100%)
        self.rpwm = PWM(Pin(PIN_RPWM))
        self.lpwm = PWM(Pin(PIN_LPWM))
        self.rpwm.freq(PWM_FREQ)
        self.lpwm.freq(PWM_FREQ)
 
        # Start with both channels at zero -- motor stopped
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)
 
        max_duty_pct = (V_SAFE_MAX / v_supply) * 100
        print(f"BTS7960 ready")
        print(f"  Supply voltage:  {v_supply}V")
        print(f"  Voltage ceiling: +/-{V_SAFE_MAX:.4f}V  (3A x Ra)")
        print(f"  Max duty cycle:  {max_duty_pct:.1f}%")
        print(f"  PWM frequency:   {PWM_FREQ}Hz")
 
    def set_voltage(self, v_cmd):
        """
        Apply a voltage command from the controller to the motor.
 
        Conversion: v_cmd [V] -> PWM duty cycle on RPWM or LPWM
 
        v_cmd is clamped to +/-V_SAFE_MAX before conversion.
        This is a second enforcement of the current limit
        (control_step() also clamps, giving two layers of protection).
 
        Positive v_cmd -> RPWM active, LPWM=0 -> piston retracts -> sinks
        Negative v_cmd -> LPWM active, RPWM=0 -> piston extends  -> rises
        Zero v_cmd     -> both zero -> coast stop
 
        Args:
            v_cmd : float -- motor voltage command [V] from control_step()
        """
        # Hard clamp -- current limit enforcement (second layer)
        v_cmd = max(V_SAFE_MIN, min(V_SAFE_MAX, v_cmd))
 
        # Scale to 16-bit duty: 0-65535 represents 0%-100% of supply voltage
        duty = int(abs(v_cmd) / self.v_supply * 65535)
 
        if v_cmd > 0:
            # Forward: retract piston, vehicle sinks, depth increases
            self.rpwm.duty_u16(duty)
            self.lpwm.duty_u16(0)
        elif v_cmd < 0:
            # Reverse: extend piston, vehicle rises, depth decreases
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(duty)
        else:
            # Zero command -- coast to stop
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(0)
 
    def stop(self):
        """
        Coast stop -- remove PWM signals, motor decelerates freely.
        Call on controlled shutdown or between missions.
        """
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)
        print("BTS7960: motor stopped (coast)")
 
    def brake(self):
        """
        Active brake -- both channels driven simultaneously.
        Shorts the motor terminals through the H-bridge,
        creating a braking torque. Stops faster than coast.
        Use for precise position holding if needed.
        """
        self.rpwm.duty_u16(65535)
        self.lpwm.duty_u16(65535)
 
    def close(self):
        """
        Safe shutdown -- stop motor and disable the H-bridge.
        Call in the finally block of main_controller.py to ensure
        motor always stops even if an exception occurs.
        """
        self.stop()
        self.r_en.value(0)
        self.l_en.value(0)
        print("BTS7960: disabled")