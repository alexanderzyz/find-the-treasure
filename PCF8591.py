#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  文件名：PCF8591.py
#  版本：V2.0
#  author: zyz,dym
# 说明：这是一个PCF8591模块的程序。
#      警告：模拟输入不能超过3.3V!
# 在这个程序中，我们使用电位计进行模拟输入和控制一个模拟电压
# 的LED灯，你可以导入这个程序到另一个程序中使用：
# import PCF8591 as ADC
# ADC.Setup(Address)  # 通过 sudo i2cdetect -y -1 可以获取到IIC的地址
# ADC.read(channal)	# 通道选择范围为0-3
# ADC.write(Value)	# 值的范围为：0-255
#####################################################
import smbus
import time

# 对应比较旧的版本如RPI V1 版本，则 "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

#通过 sudo i2cdetect -y -1 可以获取到IIC的地址
def setup(Addr):
	global address
	address = Addr

# 读取模拟量信息
def read(chn): #通道选择，范围是0-3之间
	try:
		if chn == 0:
			bus.write_byte(address,0x40)
		if chn == 1:
			bus.write_byte(address,0x41)
		if chn == 2:
			bus.write_byte(address,0x42)
		if chn == 3:
			bus.write_byte(address,0x43)
		bus.read_byte(address) # 开始进行读取转换
	except Exception as e:
		print ("Address: %s" % address)
		print (e)
	return bus.read_byte(address)

# 模块输出模拟量控制，范围为0-255
def write(val):
	try:
		temp = val # 将数值赋给temmp 变量
		temp = int(temp) # 将字符串转换为整型
		# 在终端上打印temp以查看，否则将注释掉
		bus.write_byte_data(address, 0x40, temp)
	except Exception as e:
		print ("Error: Device address: 0x%2X" % address)
		print (e)

if __name__ == "__main__":
	setup(0x48)
	while True:
		print ('AIN0 = ', read(0))
		print ('AIN1 = ', read(1))
		tmp = read(0)
		tmp = tmp*(255-125)/255+125 # 低于125时LED不会亮，所以请将“0-255”转换为“125-255”
		write(tmp)
#		time.sleep(0.3)
