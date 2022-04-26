#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# －－－－电子技术实习小作业1－－－－
#  文件名：ps2.py
#  版本：V1.0
#  author: 曾绎哲，丁一铭
# 说明：小游戏--找宝藏
#####################################################

from glob import glob
from re import X
import PCF8591 as ADC
import LCD1602
import RPi.GPIO as GPIOX
import RPi.GPIO as GPIO
import time

############全局变量##################

#当前坐标
global x
global y

#宝藏坐标
global aimedx
global aimedy
aimedx=2
aimedy=3

#地图范围
global xx
global yy
xx=5
yy=5

#蜂鸣器
makerobo_Buzzer = 33             # 无源蜂鸣器管脚定义

# 音谱定义
Tone_CL = [0, 131, 147, 165, 175, 196, 211, 248]		# 低C音符的频率
Tone_CM = [0, 262, 294, 330, 350, 393, 441, 495]		# 中C音的频率
Tone_CH = [0, 525, 589, 661, 700, 786, 882, 990]		# 高C音符的频率

# 第一首歌音谱
makerobo_song_1 = [	Tone_CH[1], Tone_CH[2], Tone_CM[5], Tone_CH[2], Tone_CH[3], Tone_CH[5], Tone_CH[4], Tone_CH[3],
                    Tone_CH[1], Tone_CH[1], Tone_CH[2], Tone_CM[5], Tone_CM[5], Tone_CM[5], Tone_CM[6], Tone_CH[1],
                    Tone_CM[6], Tone_CH[1], Tone_CH[1], Tone_CH[2], Tone_CM[5], Tone_CH[2], Tone_CH[3], Tone_CH[5],
                    Tone_CH[4], Tone_CH[3], Tone_CH[1]]
# 第1首歌的节拍，1表示1/8拍
makerobo_beat_1 = [	12, 12, 8, 12, 12, 2, 2, 2, 			
                    2, 12, 12, 8, 4, 2, 2, 2,
                    2, 2, 12, 12, 8, 12, 12, 2,
                    2, 2, 2	]

#RGB 三色LED
colors = [0x0FF000, 0x00FF00]
makerobo_R = 11
makerobo_G = 12
makerobo_B = 13

#按键
makerobo_BtnPin = 36 #轻触按键Pin端口
global judge #判断按键是否按下
judge=1
#########PS2部分######################
# 初始化
def makerobo_setup():
    ADC.setup(0x48)					# 设置PCF8591模块地址
    global makerobo_state           # 状态变量
    #蜂鸣器
    GPIOX.setmode(GPIOX.BOARD)		# 采用实际的物理管脚给GPIO口
    GPIOX.setwarnings(False)         # 关闭GPIO警告提示
    GPIOX.setup(makerobo_Buzzer, GPIOX.OUT)	# 设置无源蜂鸣器管脚为输出模式
    global makerobo_Buzz		    # 指定一个全局变量来替换gpi.pwm
    makerobo_Buzz = GPIOX.PWM(makerobo_Buzzer, 1)	# 设置初始频率为440。
    makerobo_Buzz.start(50)					# 按50%工作定额启动蜂鸣器引脚。
    #按键
    GPIO.setup(makerobo_BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # 设置BtnPin管脚为输入模式，上拉至高电平(3.3V)
    # 中断函数，当轻触按键按下时，调用detect函数
    GPIO.add_event_detect(makerobo_BtnPin, GPIO.BOTH, callback=makerobo_detect, bouncetime=200)

#RGB三色LED
#初始化
def RGB_setup(Rpin, Gpin, Bpin):
    global pins
    global p_R, p_G, p_B
    pins = {'pin_R': Rpin, 'pin_G': Gpin, 'pin_B': Bpin}
    GPIO.setmode(GPIO.BOARD)        # 采用实际的物理管脚给GPIO口
    GPIO.setwarnings(False)         # 去除GPIO口警告
    for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)   # 设置Pin模式为输出模式
        GPIO.output(pins[i], GPIO.LOW)  # 设置Pin管脚为低电平(0V)关闭LED
    p_R = GPIO.PWM(pins['pin_R'], 2000)  # 设置频率为2KHz
    p_G = GPIO.PWM(pins['pin_G'], 1999)
    p_B = GPIO.PWM(pins['pin_B'], 5000)
    # 初始化占空比为0(led关闭)
    p_R.start(0)
    p_G.start(0)
    p_B.start(0)

# 按比例缩放函数
def makerobo_pwm_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# 关闭RGB-LED灯
def makerobo_off():
    GPIO.setmode(GPIO.BOARD)   # 采用实际的物理管脚给GPIO口
    for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)     # 设置Pin模式为输出模式
        GPIO.output(pins[i], GPIO.LOW)    #  设置Pin管脚为低电平(0V)关闭LED

# 设置颜色
def makerobo_set_Color(col):   #  例如:col  = 0x112233
    R_val = (col & 0xff0000) >> 16
    G_val = (col & 0x00ff00) >> 8
    B_val = (col & 0x0000ff) >> 0

    # 把0-255的范围同比例缩小到0-100之间
    R_val = makerobo_pwm_map(R_val, 0, 255, 0, 100)
    G_val = makerobo_pwm_map(G_val, 0, 255, 0, 100)
    B_val = makerobo_pwm_map(B_val, 0, 255, 0, 100)

    p_R.ChangeDutyCycle(100-R_val)     # 改变占空比
    p_G.ChangeDutyCycle(100-G_val)     # 改变占空比
    p_B.ChangeDutyCycle(100-B_val)     # 改变占空比

# 中断函数，有按键按下时，响应中断函数
def makerobo_detect(chn):
    makerobo_Print(GPIO.input(makerobo_BtnPin))       # 打印出GPIO的状态

# 打印函数，显示出按键按下
def makerobo_Print(x):
    if x == 0:
        print('*Game Started!*')
        global judge
        judge=0

# 方向判断函数
def makerobo_direction():	
    global x
    global y
    state = ['home', 'up', 'down', 'left', 'right', 'pressed']  # 方向状态信息
    i = 0
    if ADC.read(0) <= 30:
        i = 1		# up方向
        y=y+1
    if ADC.read(0) >= 225:
        i = 2		# down方向
        y=y-1
    if ADC.read(1) >= 225:
        i = 4		# right方向
        x=x+1
    if ADC.read(1) <= 30:
        i = 3		# left方向
        x=x-1
    if ADC.read(2) == 0 and ADC.read(1) == 128:
        i = 5		# Button按下
        print('Ready to reset!')
        x=0
        y=0
        print('Reseted!')
    # home位置
    if ADC.read(0) - 125 < 15 and ADC.read(0) - 125 > -15	and ADC.read(1) - 125 < 15 and ADC.read(1) - 125 > -15 and ADC.read(2) == 255:
        i = 0
    return state[i]   # 返回状态

# 循环函数（主要功能函数）
def makerobo_loop():
    makerobo_status = ''    # 状态值赋空值
    LCD1602.init(0x27, 1)	# init(slave address, background light)
    while True:
        #LCD1602.init(0x27, 1)	# init(slave address, background light)
        makerobo_set_Color(colors[0])  # 设置颜色,初始颜色
        if judge==0:
            makerobo_tmp = makerobo_direction()   # 调用方向判断函数
            if makerobo_tmp != None and makerobo_tmp != makerobo_status:  # 判断状态是否发生改变
                print (makerobo_tmp) # 打印出方向位
                makerobo_status = makerobo_tmp # 把当前状态赋给状态值，以防止同一状态多次打印
                #显示屏显示坐标
                tempx=str(x)
                tempy=str(y)
                LCD1602.clear()
                LCD1602.write(0, 0, 'x='+tempx+' y='+tempy)
                LCD1602.write(1, 1, 'NULL')
                if abs(x)>xx or abs(y)>yy:
                    LCD1602.clear()
                    LCD1602.write(0, 0, 'x='+tempx+' y='+tempy)
                    LCD1602.write(1, 1, 'Error.Crossed!')
                    break
                else:
                    if x==aimedx and y==aimedy: #判断是否走到宝藏点
                        makerobo_set_Color(colors[1])#切换颜色，找到宝藏颜色
                        #显示器显示
                        LCD1602.clear()
                        LCD1602.write(0,0,'x='+tempx+' y='+tempy)
                        LCD1602.write(1,1,'You Got it!')
                        #播放bgm
                        while True:
                            for i in range(0, len(makerobo_song_1)):     # 播放第一首歌
                                makerobo_Buzz.ChangeFrequency(makerobo_song_1[i]) # 设置歌曲音符的频率
                                time.sleep(makerobo_beat_1[i] * 0.08)	 # 延迟一个节拍* 0.5秒的音符
                    time.sleep(0.3)
        else:
            #LCD1602.init(0x27, 1)	# init(slave address, background light)
            LCD1602.write(0,0,'Game has')
            LCD1602.write(1,1,'not started yet')
# 异常处理函数
def destroy():
    makerobo_Buzz.stop()			    # 停止蜂鸣器
    GPIOX.output(makerobo_Buzzer, 1)		# 设置蜂鸣器管脚为高电平
    GPIOX.cleanup()				        # 释放资源

    p_R.stop()      # 关闭红色PWM
    p_G.stop()      # 关闭绿色PWM
    p_B.stop()      # 关闭蓝色PWM
    makerobo_off()  # 关闭RGB-LED灯
    GPIO.cleanup()  # 释放资源
# 程序入口
if __name__ == '__main__':
    x=0
    y=0	
    RGB_setup(makerobo_R, makerobo_G, makerobo_B)#初始化RGB
    makerobo_setup()  # 初始化主要功能
    try:
        makerobo_loop() # 调用循环函数
    except KeyboardInterrupt:  	# 当按下Ctrl+C时，将执行destroy()子程序。
        destroy()   # 调用释放函数