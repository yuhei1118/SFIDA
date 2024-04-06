#m++に追記
#12期 電装班 佐藤 裕平
# piの配下に m_log.txt を置く

import pigpio
import time
import fcntl
import termios
import sys
import os

class _Getch:
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

# GPIO番号定数
LEFT_TOP_M1 = 4 # lt
LEFT_TOP_M2 = 17
RIGHT_TOP_M1 = 27 # rt
RIGHT_TOP_M2 = 22

#グローバル変数
pi = pigpio.pi()

def getkey():
    fno = sys.stdin.fileno()
    attr_old = termios.tcgetattr(fno)
    attr = termios.tcgetattr(fno)
    attr[3] = attr[3] & ~termios.ECHO & ~termios.ICANON
    termios.tcsetattr(fno, termios.TCSADRAIN, attr)
    fcntl_old = fcntl.fcntl(fno, fcntl.F_GETFL)
    fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)
    chr = 0
    try:
        c = sys.stdin.read(1)
        if len(c):
            while len(c):
                chr = (chr << 8) + ord(c)
                c = sys.stdin.read(1)
    finally:
        fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old)
        termios.tcsetattr(fno, termios.TCSANOW, attr_old)

    return chr

def moter_init():
    pi.set_mode(LEFT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(LEFT_TOP_M2, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M1, pigpio.OUTPUT)
    pi.set_mode(RIGHT_TOP_M2, pigpio.OUTPUT)

    pi.set_PWM_frequency(LEFT_TOP_M1, 50)
    pi.set_PWM_frequency(LEFT_TOP_M2, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M1, 50)
    pi.set_PWM_frequency(RIGHT_TOP_M2, 50)

    pi.set_PWM_range(LEFT_TOP_M1, 100)
    pi.set_PWM_range(LEFT_TOP_M2, 100)
    pi.set_PWM_range(RIGHT_TOP_M1, 100)
    pi.set_PWM_range(RIGHT_TOP_M2, 100)

def stop():
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)

def drange_up(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step

def drange_down(begin, end, step):
    n = begin
    while n+step > end:
        yield n
        n -= step

def step_up(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange_up(max, 0.0, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def step_down(pin, max = 100.0, t = 1.0):
    s = max / t / 10.0
    for duty in drange_down(0.0, max, s):
        pi.set_PWM_dutycycle(pin, duty)
        time.sleep(0.1)

def calibrate_fix(l_fix,r_fix,rate) :
    
    if rate == 1 :
        l_fix = l_fix - 0.025
        if l_fix <= 0 :
            l_fix = 0
        r_fix = r_fix + 0.025 
        if r_fix >= 1 :
            r_fix = 1
    
    if rate == -1 :
        l_fix = l_fix + 0.025
        if l_fix >= 1 :
            l_fix = 1
        r_fix = r_fix - 0.025
        if r_fix <= 0 :
            r_fix = 0
    
    fix =[l_fix,r_fix]
    
    return fix

def pawer_fix(p_fix,rate) :
    
    if rate == 1 :
        p_fix = p_fix + 5
    if p_fix >= 100 :
        p_fix = 100
    if p_fix <= 0 :
        p_fix = 0
    
    if rate == -1 :
        p_fix = p_fix - 5
    if p_fix >= 100 :
        p_fix = 100
    if p_fix <= 0 :
        p_fix = 0
    
    return p_fix    

def go(start, max = 100.0, t = 1.0, l_fix = 1.0, r_fix = 1.0):
    
    pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
    if start:
        s = max / t / 10.0
        for duty in drange_up(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M1, duty*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, duty*r_fix)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M1, max*l_fix)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, max*r_fix)

def back(start, max = 100.0, t = 1.0, l_fix = 1.0, r_fix = 1.0):
    pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
    pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
    if start:
        s = max / t / 10.0
        for duty in drange_up(0.0, max, s):
            pi.set_PWM_dutycycle(LEFT_TOP_M2, duty*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, duty*r_fix)
            time.sleep(0.1)
    else:
        pi.set_PWM_dutycycle(LEFT_TOP_M2, max*l_fix)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, max*r_fix)

def left(start, max = 100.0, t = 1.0, rate = 0.6, l_fix = 1.0, r_fix = 1.0):
    if rate >= 0:
        r = max
        l = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
    else:
        r = max
        l = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M1, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M2, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M2, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)

def right(start, max = 100.0, t = 1.0, rate = 0.6, l_fix = 1.0, r_fix = 1.0):
    if rate >= 0:
        l = max
        r = rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M2, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M1, r*r_fix)
    else:
        l = max
        r = - rate * max
        pi.set_PWM_dutycycle(LEFT_TOP_M2, 0)
        pi.set_PWM_dutycycle(RIGHT_TOP_M1, 0)
        if start:
            s = max / t / 10.0
            for duty in drange_up(0.0, max, s):
                if duty <= l:
                    pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
                if duty <= r:
                    pi.set_PWM_dutycycle(RIGHT_TOP_M2, r*r_fix)
        else:
            pi.set_PWM_dutycycle(LEFT_TOP_M1, l*l_fix)
            pi.set_PWM_dutycycle(RIGHT_TOP_M2, r*r_fix)

def get_moter_pawer() :
    with open("moter_pawer.txt" , "r") as mf :
        moter_pawer =mf.read()
    print("get_moter_pawer")
    return int(moter_pawer)

def write_moter_pawer(moter_pawer) :
    mp = str(moter_pawer)
    with open("moter_pawer.txt" , "w") as mf :
        moter_pawer =mf.writelines(mp)
    return 0

def get_moter_fix() :
    with open("moter_fix.txt" , "r") as mf :
        moter_fix =mf.read()
    print("get_moter_fix")
    return moter_fix

def write_moter_fix(moter_fix) :
    mf = str(moter_fix)
    with open("moter_fix.txt" , "w") as mf :
        moter_fix =mf.writelines(mf)
    return 0

#メイン処理
def main():
    moter_init()
    getch = _Getch()
    
    moter_pawer = 80
    fix = [1.0,1.0]
    
    while True:
        key = getch()
        if key == "w":
            go(0, max = moter_pawer, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=w_前進 ')
            f.close()
            print('w_前進')
        if key == "9":
            go(0, max = moter_pawer*0.9, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=9_前進_90% ') 
            f.close()
            print('9_前進_90%')
        if key == "8":
            go(0, max = moter_pawer*0.8, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=8_前進_80% ') 
            f.close()
            print('8_前進_80%')
        if key == "7":
            go(0, max = moter_pawer*0.7, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=7_前進_70% ') 
            f.close()
            print('7_前進_70%')
        if key == "6":
            go(0, max = moter_pawer*0.6, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=6_前進_60% ') 
            f.close()
            print('6_前進_60%')
        if key == "5":
            go(0, max = moter_pawer*0.5, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=5_前進_50% ') 
            f.close()
            print('5_前進_50%')
        if key == "4":
            go(0, max = moter_pawer*0.4, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=4_前進_40% ') 
            f.close()
            print('4_前進_40%')
        if key == "3":
            go(0, max = moter_pawer*0.3, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=3_前進_30% ') 
            f.close()
            print('3_前進_30%')
        if key == "2":
            go(0, max = moter_pawer*0.2, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=2_前進_20% ') 
            f.close()
            print('2_前進_20%')
        if key == "1":
            go(0, max = moter_pawer*0.1, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=1_前進_10% ') 
            f.close()
            print('1_前進_10%')
        if key == "q":
            l_pawer = moter_pawer*0.75
            left(0,l_pawer,rate = 0.3, l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=ｑ ') 
            f.close()
            print('ｑ')
        if key == "e":
            r_pawer = moter_pawer*0.75
            right(0,r_pawer,rate = 0.3,l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=e ') 
            f.close()
            print('e')
        if key == "a":
            l_pawer = moter_pawer*0.75
            left(0,l_pawer, rate = -1,l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=a ') 
            f.close()
            print('a')
        if key == "d":
            r_pawer = moter_pawer*0.75
            right(0,r_pawer, rate = -1,l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=d ') 
            f.close()
            print('d')
        if key == "s":
            b_pawer = moter_pawer*0.50
            back(0,b_pawer,l_fix = fix[0], r_fix = fix[1])
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=s ') 
            f.close()
            print('s')
        if key == ",":
            stop()
            fix = calibrate_fix(fix[0],fix[1],-1)
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=, ') 
            f.close()
            print(',')
            print(fix)
        if key == ".":
            stop()
            fix = calibrate_fix(fix[0],fix[1],1)
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=. ') 
            f.close()
            print('.')
            print(fix)
        if key == "k":
            stop()
            moter_pawer = pawer_fix(moter_pawer,-1)
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=k ') 
            f.close()
            print('k')
            print(moter_pawer)
        if key == "l":
            stop()
            moter_pawer = pawer_fix(moter_pawer,1)
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=l ') 
            f.close()
            print('l')
            print(moter_pawer)
        elif key == " ":
            stop()
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=space ') 
            f.close()
            print('space')
        elif key == "x":
            f = open('m_log.txt', 'a', encoding='UTF-8')
            f.write('comand=x ') 
            f.close()
            print('x')
            break
        
if __name__ == '__main__':
    try :
        main()
    finally :
        stop()