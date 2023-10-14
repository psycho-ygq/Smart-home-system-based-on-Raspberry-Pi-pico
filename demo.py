from machine import I2C, Pin
from ssd1306 import SSD1306_I2C
import machine
import time
import utime
import time
from machine import Pin, PWM

# 26引脚为PWM输出方式，负责调节风扇的转速
pwm = PWM(Pin(26))
pwm.freq(10000)

#10引脚作为 温度传感器的数据输入

# 11引脚是灯，光照弱时亮，光照亮时就自动熄灭
led_red = machine.Pin(11, machine.Pin.OUT)
led_red.value(0)

# 12引脚作为红外的输入，有人是高电平，没有人是低电平
radio = machine.Pin(12, machine.Pin.IN)
#调试的时候用的radio = machine.ADC(28)

light = machine.ADC(27) 
#27口是采集光照强度的

#读取温湿度传感器温度的函数
class DHT11:
    def __init__(self,pin_name):
        #DHT11上电后（DHT11上电后要等待 1S 以越过不稳定状态在此期间不能发送任何指令）
        time.sleep(1)
        self.N1 = Pin(pin_name, Pin.OUT)
        self.PinName=pin_name
        time.sleep(1)
        
    def read_data(self):
        self.__init__(self.PinName)
        data=[]
        j=0
        N1=self.N1
        N1.low()
        #微处理器的I/O设置为输出同时输出低电平，且低电平保持时间不能小于18ms（最大不得超过30ms）
        time.sleep(0.03)
        N1.high()
        N1 = Pin(self.PinName, Pin.IN)
        while N1.value()==1:
            continue
        #等待外部信号低电平结束
        while N1.value()==0:
            continue
        #等待外部信号高电平结束
        while N1.value()==1:
            continue
        #一次接受40bit数据
        while j<40:
            k=0
            while N1.value()==0:
                continue
            while N1.value()==1:
                k+=1
                if k>100:break
            if k<3:
                data.append(0)
            else:
                data.append(1)
            j=j+1
        
        j=0
        humidity_bit=data[0:8]
        humidity_point_bit=data[8:16]
        temperature_bit=data[16:24]
        temperature_point_bit=data[24:32]
        check_bit=data[32:40]
        humidity=0
        humidity_point=0
        temperature=0
        temperature_point=0
        check=0
        #温度、湿度、校验位计算
        for i in range(8):
            #湿度计算
            humidity+=humidity_bit[i]*2**(7-i)
            humidity_point+=humidity_point_bit[i]*2**(7-i)
            #温度计算
            temperature+=temperature_bit[i]*2**(7-i)
            temperature_point+=temperature_point_bit[i]*2**(7-i)
            #校验位计算
            check+=check_bit[i]*2**(7-i)
        tmp=humidity+humidity_point+temperature+temperature_point
        if tmp == check:
            return [temperature,temperature_point,humidity,humidity_point,tmp,check]
        else:
            return [0,0,0,0,0,0]


i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)    # Init I2C using I2C0 defaults, SCL=Pin(GP9), SDA=Pin(GP8), freq=100000
oled = SSD1306_I2C(128, 64, i2c)

radio_0 = 0  #初始化人传感器的状态 只有从0——1时才工作
radio_1 = 0

while True:
    dht=DHT11(10)
    data = dht.read_data()#读取温湿度的值
    oled.fill(0)
    oled.text("YGQ",32,0)
    
    oled.text("temp {}.{} degree".format(data[0],data[1]),0,16)
    #oled.text("temp {}{} degree".format(data[0],data[1]),0,32)
    
    #oled.text("humidity {}.{}RH%".format(data[2],data[3]),0,32)  读取湿度 不用
    oled.show()
    # 设定PWM的占空比，取值范围0~65535， 而32768大约是一半，也就是50%
    
    if data[0] < 27:
        pwm.duty_u16(0)
    if data[0] >= 27:
        pwm_value=10*(data[0]-27)+data[1]  #比如28.3°就是 13  假设30°是最高，从27.0——30.0°分配给32768——65535
        pwm_value=20000+pwm_value*1516
        pwm.duty_u16(pwm_value)
        
    
    print(light.read_u16())
    if light.read_u16()<10000:    
        led_red.value(0)
        oled.text("LED is off",0,32)
        oled.show()
    if light.read_u16()>=10000:
        led_red.value(1)
        oled.text("LED is on",0,32)
        oled.show()
