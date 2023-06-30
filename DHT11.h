#ifndef DHT11_H_
#define DHT11_H_
#include <msp430g2553.h>
#define delay_us(x) __delay_cycles((long)(((double)8000000)*((double)x)/1000000.0))
#define delay_ms(x) __delay_cycles((long)(((double)8000000)*((double)x)/1000.0))
//输出数据P1.5
#define    SET_DATA    P2OUT |= BIT5    //拉高数据线
#define    CLR_DATA    P2OUT &= ~BIT5   //拉低数据线
//输入数据P1.5
#define    DHT11_DATA    (P2IN&BIT5)
//保存数据数组
unsigned int rec_dat[12];
//DHT状态标志
unsigned int dht_sta;

unsigned int DHT11_Check(void);
int DHT11_ReadByte(void);
void DHT11_ReadDATA(void);

#endif /* DHT11_H_ */
