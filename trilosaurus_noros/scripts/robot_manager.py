#!/usr/bin/env python3
import numpy as np, time
import pdb
import os
import psutil
import trilobot

import RPi.GPIO as GPIO

import remote_control

import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess
class Screen:
    def __init__(self):
        self.dev = Adafruit_SSD1306.SSD1306_128_32(rst=24, i2c_bus=1, i2c_address=0x3C)
        self.w, self.h = self.dev.width, self.dev.height
        print(f'screen initialized({self.w}x{self.h})')
        #self.font = ImageFont.load_default()
        self.font = ImageFont.truetype("DejaVuSansMono.ttf", 15)
        self.img = Image.new('1', (self.w, self.h))
        self.draw = ImageDraw.Draw(self.img)

        self.page_nb, self.page = 4, 0
        
        self.dev.begin()
        self.dev.clear()
        self.dev.display()

    def display(self, m):
        self.draw.rectangle((0, 0, self.w, self.h), outline=0, fill=0)
        x, y0,y1 = 0, -2, 18
        if self.page == 0:
            self.draw.text((x, y0), f'{m.hostname}',  font=self.font, fill=255)
            cur_time = time.strftime('%H:%M:%S', time.localtime())
            self.draw.text((x, y1), f"{cur_time}",  font=self.font, fill=255)
        elif self.page == 1:
            self.draw.text((x, y0), f'Host {m.hostname}',  font=self.font, fill=255)
            self.draw.text((x, y1), f'{m.ip}',  font=self.font, fill=255)
        elif self.page == 2:
            self.draw.text((x, y0), f'{m.freq}Hz ({m.freq_real:.1f})',  font=self.font, fill=255)
            self.draw.text((x, y1), f'CPU {m.cpu_load: .02f}%',  font=self.font, fill=255)
        elif self.page == 3:
            self.draw.text((x, y0), f'Disk use {m.disk_free:4.01f}%',  font=self.font, fill=255)
            self.draw.text((x, y1), f'Temp {m.temp:4.01f}C',  font=self.font, fill=255)
        self.dev.image(self.img)
        self.dev.display()

    def next_page(self):
        self.page = (self.page+1)%self.page_nb

    def prev_page(self):
        self.page = (self.page-1)%self.page_nb
        



        
class MyTrilobot(trilobot.Trilobot):
    def __init__(self):
        trilobot.Trilobot.__init__(self)
        self.bs = [trilobot.BUTTON_A, trilobot.BUTTON_B, trilobot.BUTTON_X, trilobot.BUTTON_Y]
        self.but_states = [False]*len(self.bs)

    def start(self):
        self.clear_underlighting()

    def _read_buttons(self):
        but_states = np.logical_not(np.array([GPIO.input(self.buttons[_b]) for _b in self.bs], dtype=bool))
        self.but_changed = np.logical_xor(self.but_states, but_states)
        self.but_states = but_states
        
    def loop(self):
        self._read_buttons()
        self.set_button_led(trilobot.BUTTON_A, self.but_states[trilobot.BUTTON_A])



        

class RobotManager:
    def __init__(self):
        self.tbot = MyTrilobot()
        self.joystick = remote_control.create_mocute_controller(stick_deadzone_percent=0.1)
        self.screen = Screen()
        
        
    def start(self):
        self.tbot.start()
        self.joystick.connect()
       
        self.hostname = subprocess.check_output("hostname", shell=True ).decode()
        self.ip = subprocess.check_output("hostname -I | cut -d\' \' -f1", shell=True ).decode()
        self.freq_real = 0.
        
    def update_sys(self, loop_nb):
        step = loop_nb % 15 
        if step == 0:
            #cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
            #cmd = "grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage}'"
            #self.cpu_load = float(subprocess.check_output(cmd, shell = True ).decode())
            self.cpu_load = psutil.cpu_percent()
        elif step == 7:
            self.disk_free =  psutil.disk_usage('/').percent
        elif step == 8:
            self.temp = subprocess.check_output("vcgencmd measure_temp", shell = True ).decode() # "temp=39.9'C\n"
            self.temp = float(self.temp.split('=')[-1][:-3])                 # fetch the number
            #pdb.set_trace()
            #print(self.temp)

    def display_idle(self, now, om=.5):
        tbot.fill_underlighting(math.sin(now*om) * 127+127, 0, 0)
        
        
    def run(self, freq_hz=30.):
        self.freq = freq_hz
        self.dt_loop = 1/freq_hz
        last_loop_start = time.clock_gettime(time.CLOCK_MONOTONIC) - self.dt_loop
        loop_cnt = 0
        while True:
            loop_start = time.clock_gettime(time.CLOCK_MONOTONIC)
            self.loop(loop_cnt)
            #_next = now + dt_loop
            proc_end = time.clock_gettime(time.CLOCK_MONOTONIC) 
            dt_proc =  proc_end - loop_start
            dt_sleep = max(0, self.dt_loop - dt_proc)
            self.freq_real = 1./(loop_start - last_loop_start)
            last_loop_start = loop_start
            #print(f' freq:{1./loop_elapsed:.1f}hz, (busy/sleep: {loop_busy:.3f}/{sleep_time:.3f}s)')
            time.sleep(dt_sleep)
            loop_cnt += 1


    def loop(self, loop_cnt, auto_cmd=None):
        if not self.joystick.is_connected():
            self.joystick.reconnect(10, True)
        try:
            self.joystick.update(debug=False)
        except RuntimeError:
            self.tbot.disable_motors()
        if self.joystick.is_connected() or auto_cmd is not None:
            if auto_cmd is None:
                sp_lvel, sp_rvel = -0.04*self.joystick.read_axis("LY"), -0.8*self.joystick.read_axis("RX")
            else:
                sp_lvel, sp_rvel = auto_cmd
            #KL, KR = 17.5, -0.8
            KL, KR = 17.5, -1.3
            lpwm, rpwm = KL*sp_lvel + KR*sp_rvel, KL*sp_lvel - KR*sp_rvel
            lpwm, rpwm = np.clip([lpwm, rpwm], -1, 1)
            #print(f'{sp_lvel} m/s, {np.rad2deg(sp_rvel):.1f} deg/s -> {lpwm} {rpwm}')
            self.tbot.set_motor_speeds(lpwm, rpwm)
        else:
            self.tbot.set_motor_speeds(0, 0)
            #pdb.set_trace()
        self.update_sys(loop_cnt)
        self.tbot.loop()
        if self.tbot.but_states[0] and self.tbot.but_changed[0]:
            self.screen.next_page()
        elif self.tbot.but_states[1] and self.tbot.but_changed[1]:
            self.screen.prev_page()
                
        if loop_cnt%5==0:
            self.screen.display(self)
        


def main(freq_hz=30):
    r = RobotManager()
    r.start()
    r.run(freq_hz)
            
if __name__ == '__main__':
    main()
