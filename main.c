#include "msp430.h"
#include "DHT11.h"
#define HIGH P2OUT|=BIT5;
#define LOW P2OUT&=~BIT5;
/*设置界面的标志*/
int sign_setup_change, sign_setup_class, sign_setup_wei, sign_setup_flash;
/*数码管的变量*/
int show[6] = { 14,14,14,14,14,14 };//初始化为全部熄灭
const unsigned int number[] = { //共阴
    0x3F,  //"0"
    0x06,  //"1"
    0x5B,  //"2"
    0x4F,  //"3"
    0x66,  //"4"
    0x6D,  //"5"
    0x7D,  //"6"
    0x07,  //"7"
    0x7F,  //"8"
    0x6F,  //"9"
    0x77,  //10:"A"
    0x39,  //11:"C"
    0x76,  //12:"H"
    0x40,  //13:"-"
    0x00,  //14:熄灭
    0x3E   //15："U"
};
const unsigned int wei1[6] = { BIT0,BIT1,BIT2,BIT3,BIT4,BIT5 };
int sign_point2, sign_point4, sign_point5;//dp对应的点
int sign_5s_count, sign_5s_flag;//5s的计数和标志

//int sign_switch0, sign_switch1, sign_switch2, sign0_5s, sign1_5s, sign_usb, sign_usb1;
/*DHT11相关的变量*/
int buf[5] = { 0x00,0x00,0x00,0x00,0x00 };
unsigned long sum = 0;
int num, i = 0;
unsigned int rec_dat[12];//保存数据数组
unsigned int dht_sta;//DHT状态标志
#define delay_us(x) __delay_cycles((long)(((double)8000000)*((double)x)/1000000.0))
#define delay_ms(x) __delay_cycles((long)(((double)8000000)*((double)x)/1000.0))
/*数字时钟相关的变量*/
int halfs, second0 = 50, minute0 = 59, hour0 = 12, day0 = 1, month0 = 1, year0 = 2023, week0 = 1, alarm_hour0 = 6, alarm_minute0;
int clock_time[6] = { 1,2,0,0,0,0 }, clock_date[6] = { 1,14,0,1,0,1 }, clock_year[6] = { 13,2,0,2,3,13 }, clock_alarm[6] = { 10,14,0,6,0,0 };
int typec_flag;
/*DHT11的相关函数*/
unsigned int DHT11_Check(void) {
    unsigned int retry = 0;
    P2DIR |= BIT5;
    P2OUT |= BIT5;
    delay_us(2);
    P2OUT &= ~BIT5;//拉低20ms
    delay_ms(20);
    P2OUT |= BIT5;//拉高30us
    delay_us(30);
    P2DIR &= ~BIT5;    //转换为输入端口
    P2REN |= BIT5;     //启用上下拉电阻
    P2OUT |= BIT5;     //选择上拉电阻
    while ((!(P2IN & BIT5)) && (retry < 16)) {  //等待低电平响应过去
        retry++;
        delay_us(5);
    }
    if (retry == 20)    return 0;
    else    retry = 0;
    while ((P2IN & BIT5) && (retry < 16)) {      //等待高电平响应过去
        retry++;
        delay_us(5);
    }
    if (retry >= 20) return 0;
    else return 1;
}
int DHT11_ReadByte(void) {     //接收1Byte数据
    int dat = 0;
    unsigned int retry = 0, i = 0, temp = 0;
    for (i = 0; i < 8; i++) {
        retry = 0;
        while ((!(P2IN & BIT5)) && (retry < 20)) {   //等待50us低电平过去
            retry++;
            delay_us(5);
        }
        if (retry >= 20) break;
        delay_us(45);               //延时45us
        temp = 0;
        retry = 0;
        if (P2IN & BIT5) {              //26~28us表示bit0，70us表示bit1
            temp = 1;
            while ((P2IN & BIT5) && (retry < 5)) {
                retry++;
                delay_us(5);
            }
        }
        dat <<= 1;
        dat |= temp;
    }
    return dat;
}
void DHT11_ReadDATA(void) {     //接收40位数据
    int humidity_int = 0x00, temprature_int = 0x00, humidity_float = 0x00, temprature_float = 0x00;
    unsigned int i = 0, retry = 0;
    if (DHT11_Check())
    {
        dht_sta = 1;
        for (i = 0; i < 5; i++)
        {
            buf[i] = DHT11_ReadByte();
        }
        retry = 0;
        while ((!(P2IN & BIT5)) && (retry < 10))   //等待50us低电平过去
        {
            retry++;
            delay_us(5);
        }
        P2REN &= ~BIT5;    //关闭上下拉电阻
        P2DIR |= BIT5;     //设为输出端口
        P2OUT |= BIT5;
        sum = buf[0] + buf[1] + buf[2] + buf[3];
        if (buf[4] == (unsigned int)sum)
        {
            humidity_int = buf[0];
            humidity_float = buf[1];
            temprature_int = buf[2];
            temprature_float = buf[3];
        }
    }
    else
    {
        humidity_int = 0x00;
        temprature_int = 0x00;
        dht_sta = 0;
    }
    //数据处理
    rec_dat[0] = humidity_int / 10 % 10;
    rec_dat[1] = humidity_int % 10;
    rec_dat[2] = humidity_float / 10 % 10;
    rec_dat[3] = humidity_float % 10;
    rec_dat[4] = 12;//'H';
    rec_dat[5] = temprature_int / 10 % 10;
    rec_dat[6] = temprature_int % 10;
    rec_dat[7] = temprature_float % 10;
    rec_dat[8] = temprature_float / 10 % 10;
    rec_dat[9] = 0xdf;    //0xdf= '°'
    rec_dat[10] = 11;//'C';
    rec_dat[11] = '\0';
}
/*显示的函数*/
void led_show(int* n) {      //此函数有延时函数
    P1OUT |= (BIT1 + BIT2 + BIT3 + BIT0 + BIT4 + BIT5);
    for (i = 0; i < 6; i++) {
        /*调整段选*/
        num = n[i];
        P3OUT = number[num];
        if (sign_point2 != 0) {
            if (i == 1)
                P3OUT |= BIT7;
        }
        if (sign_point4 != 0) {
            if (i == 3)
                P3OUT |= BIT7;
        }
        if (sign_point5 != 0) {
            if (i == 4)
                P3OUT |= BIT7;
        }
        /*调整位选*/
        if (((sign_setup_change == 1) || (sign_setup_change == 3)) && (sign_setup_wei == 6 - i)) {
            if (sign_setup_flash % 2 == 0)
                P1OUT &= ~wei1[i];
        }
        else
            P1OUT &= ~wei1[i];
        __delay_cycles(1000);
        P1OUT |= wei1[i];
    }
}
/*数组更新*/
void time_renew(int* show, int* clock_time) {//把时间数组传到显示数组
    for (i = 0; i < 6; i++)
        show[i] = clock_time[i];
}
void date_renew(int* show, int* clock_date) {
    for (i = 0; i < 6; i++)
        show[i] = clock_date[i];
}
void year_renew(int* show, int* clock_year) {
    for (i = 0; i < 6; i++)
        show[i] = clock_year[i];
}
void alarm_renew(int* show, int* clock_alarm) {
    for (i = 0; i < 6; i++)
        show[i] = clock_alarm[i];
}
/*计算具体时间和更新show相关数组（非显示）*/
void plus(int setup_class, int setup_wei) {
    if (setup_class == 1) {     //time:     20:20:20
        switch (setup_wei) {
        case 1: second0++; break;
        case 2: second0 += 10; break;
        case 3: minute0++; break;
        case 4: minute0 += 10; break;
        case 5: hour0++; break;
        case 6: hour0 += 10; break;
        case 0: break;
        }
    }
    else if (setup_class == 2) {    //date:     4*06:15
        switch (setup_wei) {
        case 1: day0++; break;
        case 2: day0 += 10; break;
        case 3: month0++; break;
        case 4: month0 += 10; break;
        case 5:  break;//sign_setup_wei = 1;
        case 6:  break;//sign_setup_wei = 1;
        case 0: break;
        }
    }
    else if (setup_class == 3) {    //year:     -2023-
        switch (setup_wei) {
        case 1: break;
        case 2: year0++; break;
        case 3: year0 += 10; break;
        case 4: year0 += 100; break;
        case 5: year0 += 1000; break;
        case 6:  break;//sign_setup_wei = 1;
        case 0:  break;//sign_setup_wei = 1;
        }
    }
    else if (setup_class == 0) {
        /*switch (setup_wei) {
        case 1: second0++; break;
        case 2: second0 += 10; break;
        case 3: minute0++; break;
        case 4: minute0 += 10; break;
        case 5: hour0++; break;
        case 6: hour0 += 10; break;
        case 0: break;
        }*/
        //sign_setup_class = 1;
    }
    else if (setup_class == 10) {
        switch (setup_wei) {
        case 1: alarm_minute0++; break;
        case 2: alarm_minute0 += 10; break;
        case 3: alarm_hour0++; break;
        case 4: alarm_hour0 += 10; break;
        case 5:  break;//sign_setup_wei = 1;
        case 6:  break;//sign_setup_wei = 1;
        case 0:  break;
        }
    }
}
void minus(int setup_class, int setup_wei) {
    if (setup_class == 1) {     //time:     20:20:20
        switch (setup_wei) {
        case 1: second0--; break;
        case 2: second0 -= 10; break;
        case 3: minute0--; break;
        case 4: minute0 -= 10; break;
        case 5: hour0--; break;
        case 6: hour0 -= 10; break;
        case 0: break;
        }
    }
    else if (setup_class == 2) {    //date:     4*06:15
        switch (setup_wei) {
        case 1: day0--; break;
        case 2: day0 -= 10; break;
        case 3: month0--; break;
        case 4: month0 -= 10; break;
        case 5:  break;//sign_setup_wei = 1;
        case 6:  break;//sign_setup_wei = 1;
        case 0: break;
        }
    }
    else if (setup_class == 3) {    //year:     -2023-
        switch (setup_wei) {
        case 1: break;
        case 2: year0--; break;
        case 3: year0 -= 10; break;
        case 4: year0 -= 100; break;
        case 5: year0 -= 1000; break;
        case 6:  break;//sign_setup_wei = 1;
        case 0:  break;//sign_setup_wei = 1;
        }
    }
    else if (setup_class == 0) {
        /*switch (setup_wei) {
        case 1: second0++; break;
        case 2: second0 += 10; break;
        case 3: minute0++; break;
        case 4: minute0 += 10; break;
        case 5: hour0++; break;
        case 6: hour0 += 10; break;
        case 0: break;
        }*/
        //sign_setup_class = 1;
    }
    else if (setup_class == 10) {
        switch (setup_wei) {
        case 1: alarm_minute0--; break;
        case 2: alarm_minute0 -= 10; break;
        case 3: alarm_hour0--; break;
        case 4: alarm_hour0 -= 10; break;
        case 5:  break;//sign_setup_wei = 1;
        case 6:  break;//sign_setup_wei = 1;
        case 0:  break;
        }
    }
}
int get_weekday(int y, int m, int d) {
    int w;
    // 如果m是1或2，就把它看成上一年的13或14月，并把y减1。
    if (m == 1 || m == 2) {
        m += 12;
        y--;
    }
    // 使用基姆拉尔森计算公式，w = (d+2*m+3*(m+1)/5+y+y/4-y/100+y/400) mod 7，计算出w的值，并返回。
    w = (d + 2 * m + 3 * (m + 1) / 5 + y + y / 4 - y / 100 + y / 400) % 7;
    return w;
}
int get_weeknum(int y, int m, int d) {
    int n;
    int a[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) {
        a[1] = 29;
    }
    int sum = 0;
    // 从1到m-1，把每个月的天数加到sum
    for (i = 0; i < m - 1; i++) {
        sum += a[i];
    }
    sum += d;
    int offset = 0;//初始为0，用来记录一月一日是星期几
    offset = get_weekday(y, 1, 1); // 调用get_weekday函数，传入y,1,1作为参数，得到offset的值
    n = 1;// 初始为1，记录第几周。
    // 从1到sum，每次判断当前的天数是否是周一，并且不是一月一日。
    for (i = 1; i <= sum; i++) {
        if ((i - offset) % 7 == 0 && i != offset) {
            n++;// 如果是周一，并且不是一月一日，就把n加1。
        }
    }
    return n;
}
void calculate(void) {
    //对时间进行计算和调整
    if (second0 >= 60) {
        second0 %= 60;
        minute0++;
    }
    if (second0 < 0) {
        second0 = 59;
        minute0--;
    }
    if (minute0 >= 60) {
        minute0 %= 60;
        hour0++;
    }
    if (minute0 < 0) {
        minute0 = 59;
        hour0--;
    }
    if (hour0 >= 24) {
        hour0 %= 24;
        day0++;
    }
    if (hour0 < 0) {
        hour0 = 23;
        day0--;
    }
    if (alarm_minute0 >= 60) {
        alarm_minute0 %= 60;
        hour0++;
    }
    if (alarm_minute0 < 0) {
        alarm_minute0 = 59;
        hour0--;
    }
    if (alarm_hour0 >= 24) {
        alarm_hour0 %= 24;
        day0++;
    }
    if (alarm_hour0 < 0) {
        alarm_hour0 = 23;
        day0--;
    }
    if (day0 < 0) {
        if (((year0 % 4 == 0 && year0 % 100 != 0) || year0 % 400 == 0) && month0 % 12 == 3) {
            day0 = 29;
            month0--;
        }
        else {
            if (month0 % 12 == 2 || month0 % 12 == 4 || month0 % 12 == 6 || month0 % 12 == 8 || month0 % 12 == 9 || month0 % 12 == 11 || month0 % 12 == 1) {
                day0 = 31;
                month0--;
            }
            else if (month0 % 12 == 5 || month0 % 12 == 7 || month0 % 12 == 10 || month0 % 12 == 0) {
                day0 = 30;
                month0--;
            }
            else if (month0 % 12 == 3) {
                day0 = 28;
                month0--;
            }
        }
    }
    if (((year0 % 4 == 0 && year0 % 100 != 0) || year0 % 400 == 0) && month0 % 12 == 2) {
        if (day0 >= 30) {
            day0 %= 29;
            month0++;
        }
    }
    else {//不是闰年
        if (month0 % 12 == 1 || month0 % 12 == 3 || month0 % 12 == 5 || month0 % 12 == 7 || month0 % 12 == 8 || month0 % 12 == 10 || month0 % 12 == 0) {
            if (day0 >= 32) {
                day0 %= 31;
                month0++;
            }
        }
        else if (month0 % 12 == 4 || month0 % 12 == 6 || month0 % 12 == 9 || month0 % 12 == 11) {
            if (day0 >= 31) {
                day0 %= 30;
                month0++;
            }
        }
        else if (month0 % 12 == 2) {
            if (day0 >= 29) {
                day0 %= 28;
                month0++;
            }
        }
    }
    if (month0 >= 13) {
        year0++;
        month0 %= 12;
    }
    if (month0 < 0) {
        year0--;
        month0 = 12;
    }
    //更新显示的数组
    clock_alarm[2] = alarm_hour0 / 10 % 10;
    clock_alarm[3] = alarm_hour0 % 10;
    clock_alarm[4] = alarm_minute0 / 10 % 10;
    clock_alarm[5] = alarm_minute0 % 10;
    clock_time[0] = hour0 / 10 % 10;
    clock_time[1] = hour0 % 10;
    clock_time[2] = minute0 / 10 % 10;
    clock_time[3] = minute0 % 10;
    clock_time[4] = second0 / 10 % 10;
    clock_time[5] = second0 % 10;
    week0 = get_weeknum(year0, month0, day0);
    if (week0 < 10) {
        clock_date[0] = week0;
        clock_date[1] = 14;
    }
    else {
        clock_date[0] = week0 / 10 % 10;
        clock_date[1] = week0 % 10;
    }
    clock_date[2] = month0 / 10 % 10;
    clock_date[3] = month0 % 10;
    clock_date[4] = day0 / 10 % 10;
    clock_date[5] = day0 % 10;
    clock_year[0] = 13;
    clock_year[1] = year0 / 1000 % 10;
    clock_year[2] = year0 / 100 % 10;
    clock_year[3] = year0 / 10 % 10;
    clock_year[4] = year0 % 10;
    clock_year[5] = 13;
}
void init_button() {
    //配置开关的外部中断下降沿触发
    P2DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);  // 配置P2.0~P2.4为输入
    P2REN |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4);   // 启用P2.0~P2.4的上下拉电阻
    P2OUT |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4);   // 配置P2.0~P2.4的上拉电阻
    P2IES |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4);   // 设置P2.0~P2.4为下降沿触发
    P2IFG &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);  // 清除P2.0~P2.4的中断标志
    P2IE |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4);    // 使能P2.0~P2.4的中断
}
//Lowpower init
void init_timer()
{
    // 配置定时器
    TA0CTL = TASSEL_1 + MC_1; // 设置定时器时钟为ACLK，连续计数模式
    TA0CCTL0 |= CCIE; // 使能比较中断
    TA0CCR0 = 32768 / 8; // 设置计数值为10000
}
void timer1_init(void) {
  TA1CTL = TASSEL_1 + MC_1; // 选择ACLK作为时钟源，时钟分频系数为8，计数模式为增计数
  TA1CCR0 = 100; // 设定计数器上限，产生1s的定时器中断
  TA1CCTL0 = CCIE; // 使能计时器中断
}
void init_LPM_IO()
{
    P1DIR = 0x00; // 配置P1口为输入
    P1OUT = 0xFF; // 设置P1口上拉电阻
    P1REN = 0xFF - BIT6;

    P2DIR &= ~BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;// 配置P2口为输入
    P2OUT |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5; // 设置P2口上拉电阻
    P2REN |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;
    //DHT11 DATA数据引脚，电压1.5V，手动调零；
    P2DIR |= BIT5;
    P2OUT &= ~BIT5;
    P3DIR = 0x00; // 配置P3口为输入
    P3OUT = 0xFF; // 设置P3口上拉电阻
    P3REN = 0xFF;

    //蜂鸣器测试
        //P1DIR |=BIT7;
}
void init_LPM_clock()
{
    BCSCTL1 &= ~XTS; // LFXTCLK 0:Low Freq
    BCSCTL2 |= SELM_3; // 选择 MCLK 时钟源为 LFXT1
    BCSCTL3 |= LFXT1S_0; // 选择 LFXT1 时钟源为 32768外部晶振
    BCSCTL1 |= DIVA_0; // 将 ACLK 分频为 1
}
void init_IO()
{
    P2DIR |= BIT5;//配置DATA
    P1DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);//初始化六个位选
    P3DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);//初始化8个段选
    P3REN &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7);//初始化8个段选
    P1REN &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);//初始化6个
}
void init_clock_timer_highpower()
{
    DCOCTL = CALDCO_8MHZ;       // 设置MCLK为8MHz
    BCSCTL1 = CALBC1_8MHZ;      // 校准时钟
    init_timer();
}

void main(void) {
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    init_LPM_clock();
    init_timer();
    init_LPM_IO();
    time_renew(show, clock_time);
    __bis_SR_register(GIE); // 进入LPM3模式

    while (1) {
        /*time_renew(show, clock_time);
        led_show(show);*/
        if (sign_setup_change == 0) {   //普通界面
            if (sign_5s_flag == 1)
                led_show(show);
            else {
                sign_point2 = 1;
                sign_point4 = 1;
                sign_point5 = 0;
                time_renew(show, clock_time);
                led_show(show);
            }
        }
        else if (sign_setup_change == 1) {   //设置时间界面
            if (sign_setup_class == 1) {
                time_renew(show, clock_time);
                led_show(show);
            }
            else if (sign_setup_class == 2) {
                date_renew(show, clock_date);
                led_show(show);
            }
            else if (sign_setup_class == 3) {
                year_renew(show, clock_year);
                led_show(show);
            }
            else if (sign_setup_class == 0) {
                //time_renew(show, clock_time);
                led_show(show);
                //sign_setup_class = 1;
            }
        }
        else if (sign_setup_change == 2) {  //进入闹钟显示界面
            alarm_renew(show, clock_alarm);
            led_show(show);
        }
        else if (sign_setup_change == 3) {  //进入设置闹钟的界面
            alarm_renew(show, clock_alarm);
            led_show(show);
        }
    }
}

#pragma vector = TIMER0_A0_VECTOR//TA0CCR0中断服务函数
__interrupt void TIMER0_A0_ISR(void) {
    //typec检测：P1.6口高电平，type——C插入
    TA0CCTL0 &= ~CCIFG;

    halfs++;
    if (halfs == 8) {
        halfs = 0;
        second0++;
        if (sign_5s_flag == 1)
            sign_5s_count++;
        if (sign_5s_count == 5) {
            sign_5s_flag = 0;
            sign_5s_count = 0;
        }
        calculate();
        //Beep
        if(minute0==0 && second0==0){
          P1DIR |=BIT7;
          P1REN &=~BIT7;
          timer1_init();          
          }
        if(minute0==0 && second0==2){
          P1DIR &=~BIT7;
          TA1CCTL0 &=~ CCIE;
        }
        }
        //
    if ((sign_setup_change == 1) || (sign_setup_change == 3))
        sign_setup_flash++;
    if (sign_setup_flash == 1000)
        sign_setup_flash = 0;





    if ((P1IN & BIT6) && (typec_flag == 1)) {
        typec_flag = 0;
        init_IO();
        init_button();
        init_clock_timer_highpower();
        __bic_SR_register_on_exit(LPM3_bits); // 清除低功耗模式 3（LPM3）位
    }
    //typec检测：P1.6口低电平，type——C拔出
    if ((P1IN == 0xBF) && (typec_flag == 0)) {
        typec_flag = 1;
        init_LPM_clock();
        init_timer();
        init_LPM_IO();
        P2IE &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4);
        __bis_SR_register(LPM3_bits + GIE); // 进入LPM3模式

    }
}
#pragma vector = TIMER1_A0_VECTOR // 定时器1中断向量
__interrupt void timer1_isr(void) {
  P1OUT ^=BIT7;
}
/*
#pragma vector=PORT1_VECTOR// 外部中断服务程序
__interrupt void typec_Interrupt(void) {
  P1IFG &= ~BIT6;
}
*/
#pragma vector=PORT2_VECTOR// 外部中断服务程序
__interrupt void Switch_Interrupt(void) {
    //__delay_cycles(900); // 延时一段时间以消除按键抖动
    //switch_3s_flag = 1;
    if (!(P2IN & BIT2)) { // 判断“下”按键是否按下,显示日期
        if (sign_setup_change == 0) { //位于普通功能，主界面转换
            sign_point4 = 1;
            sign_point2 = 0;
            sign_point5 = 0;
            sign_5s_flag = 1;
            sign_5s_count = 0;
            date_renew(show, clock_date);
        }
        else if (sign_setup_change == 1) { //在设置时间，日期，年份的界面
            //功能：对应位减1
            minus(sign_setup_class, sign_setup_wei);
            calculate();
        }
        else if (sign_setup_change == 2) { //显示闹钟的界面
            //无效按键
        }
        else if (sign_setup_change == 3) { //设置闹钟
            //功能:闹钟减1
            minus(10, sign_setup_wei);
            calculate();
        }
    }
    if (!(P2IN & BIT1)) { // 判断“中”按键是否按下,显示时间
        sign_setup_change++;
        if (sign_setup_change == 4)
            sign_setup_change = 0;
        /*sign_point2 = 1;
        sign_point4 = 1;
        sign_point5 = 0;
        time_renew(show, clock_time);*/
    }
    if (!(P2IN & BIT4)) { // 判断“上”按键是否按下,显示年份
        if (sign_setup_change == 0) { //位于普通功能，主界面转换
            sign_point2 = 0;
            sign_point4 = 0;
            sign_point5 = 0;
            sign_5s_flag = 1;
            sign_5s_count = 0;
            year_renew(show, clock_year);
        }
        else if (sign_setup_change == 1) { //在设置时间，日期，年份的界面
            //功能：对应位加1
            plus(sign_setup_class, sign_setup_wei);
            calculate();
        }
        else if (sign_setup_change == 2) { //显示闹钟的界面
            //无效按键
        }
        else if (sign_setup_change == 3) { //设置闹钟
            //功能:闹钟加一
            plus(10, sign_setup_wei);
            calculate();
        }
    }
    if (!(P2IN & BIT3)) { // 判断“右”按键是否按下,湿度
        if (sign_setup_change == 0) {   //位于普通功能，主界面转换
            sign_point2 = 0;
            sign_point4 = 0;
            sign_point5 = 1;
            sign_5s_flag = 1;
            sign_5s_count = 0;
            DHT11_ReadDATA();
            show[0] = 12;
            show[1] = 14;
            show[2] = 14;
            show[3] = rec_dat[0];
            show[4] = rec_dat[1];
            show[5] = rec_dat[2];
        }
        else if (sign_setup_change == 1) { //在设置时间，日期，年份的界面
            //功能:类的选择
            if (sign_setup_class == 0) {    //时间
                sign_point2 = 1;
                sign_point4 = 1;
                sign_point5 = 0;
                time_renew(show, clock_time);
            }
            else if (sign_setup_class == 1) {   //日期
                sign_point4 = 1;
                sign_point2 = 0;
                sign_point5 = 0;
                date_renew(show, clock_date);
            }
            else if (sign_setup_class == 2) {   //年份
                sign_point2 = 0;
                sign_point4 = 0;
                sign_point5 = 0;
                year_renew(show, clock_year);
            }
            sign_setup_class++;
            if (sign_setup_class == 3)
                sign_setup_class = 0;
        }
        else if (sign_setup_change == 2) { //显示闹钟的界面
            //无效按键
        }
        else if (sign_setup_change == 3) { //设置闹钟
            //功能：无效按键
        }
    }
    if (!(P2IN & BIT0)) { // 判断“左”按键是否按下,温度
        if (sign_setup_change == 0) {   //位于普通功能，主界面转换
            sign_point2 = 0;
            sign_point4 = 0;
            sign_point5 = 1;
            sign_5s_flag = 1;
            sign_5s_count = 0;
            DHT11_ReadDATA();
            show[0] = 11;
            show[1] = 14;
            show[2] = 14;
            show[3] = rec_dat[5];
            show[4] = rec_dat[6];
            show[5] = rec_dat[7];
        }
        else if (sign_setup_change == 1) { //在设置时间，日期，年份的界面
            //功能:位的选择
            sign_setup_wei++;
            if (sign_setup_wei == 7)
                sign_setup_wei = 0;
        }
        else if (sign_setup_change == 2) { //显示闹钟的界面
            //无效按键
        }
        else if (sign_setup_change == 3) { //设置闹钟
            //功能：选择具体的位数
            sign_setup_wei++;
            if (sign_setup_wei == 7)
                sign_setup_wei = 0;
        }
    }
    //if ((P2IN & BIT4) || (P2IN & BIT3) || (P2IN & BIT2) || (P2IN & BIT1) || (P2IN & BIT0))
    P2IFG &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4); // 清除中断标志

}


