#ifndef __LINUX_AXP_CFG_H_
#define __LINUX_AXP_CFG_H_
#include "axp-mfd.h"

/*�豸��ַ*/
/*
	һ�㲻�ı䣺
	AXP22:0x34
*/
#define	AXP_DEVICES_ADDR	(0x68 >> 1)
/*i2c���������豸��:���忴��ʹ��ƽ̨Ӳ��������*/
#define	AXP_I2CBUS			2
/*��ԴоƬ��Ӧ���жϺţ����忴��ʹ�õ�ƽ̨Ӳ�������ӣ�
�ж���nmi����cpu����·irq����gpio*/
#define AXP_IRQNO			64

/*��ʼ����·�������λmV��0��Ϊ�ر�*/
/*
	ldo1��
		��Ӳ�����������ѹ������Ĳ��ˣ�ֻ���������ʾ��ѹ��
*/
#define AXP_LDO1_VALUE			3000
/*
	ldo2��
		AXP22:700~3300,100/step
*/
#define AXP_LDO2_VALUE		3300
/*
	ldo3��
		AXP22:700~3300,100/step
*/
#define AXP_LDO3_VALUE		1000
/*
	ldo4��
		AXP22:700~3300,100/step
*/
#define AXP_LDO4_VALUE		1800
/*
	ldo5��
		AXP22:700~3300,100/step
*/
#define AXP_LDO5_VALUE		1800
/*
	ldo6��
		AXP22:700~3300,100/step
*/
#define AXP_LDO6_VALUE		2800
/*
	ldo7��
		AXP22:700~3300,100/step
*/
#define AXP_LDO7_VALUE		3300
/*
	ldo8��
		AXP22:700~3300,100/step
*/
#define AXP_LDO8_VALUE		3000
/*
	ldo9��
		AXP22:700~3300,100/step
*/
#define AXP_LDO9_VALUE		3000
/*
	ldo10��
		AXP22:700~3300,100/step
*/
#define AXP_LDO10_VALUE		1800
/*
	ldo11��
		AXP22:700~3300,100/step
*/
#define AXP_LDO11_VALUE		1200
/*
	ldo12��
		AXP22:700~1400,100/step
*/
#define AXP_LDO12_VALUE		1000

/*
	DCDC1:
		AXP22:1600~3400,100/setp
*/
#define AXP_DCDC1_VALUE		3000
/*
	DCDC2��
		AXP22:600~1540��20/step
*/
#define AXP_DCDC2_VALUE		1200
/*
	DCDC3��
		AXP22:600~1860��20/step
*/
#define AXP_DCDC3_VALUE		1200
/*
	DCDC4��
		AXP22:600~1540��20/step
*/
#define AXP_DCDC4_VALUE		1500
/*
	DCDC5��
		AXP22:1000~2550��50/step
*/
#define AXP_DCDC5_VALUE		1500

#define AXP_GPIOLDO0_VALUE 	2800

#define AXP_GPIOLDO1_VALUE      3300

/*���������mAh������ʵ�ʵ�����������壬�Կ��ؼƷ�����˵
�����������Ҫ����������*/
#define BATCAP				6000

/*��ʼ��������裬m����һ����100~200֮�䣬������ø���ʵ��
���Գ�����ȷ���������Ǵ򿪴�ӡ��Ϣ�����ӵ���պù̼���
�ϵ�أ����ӳ����������������1���Ӻ󣬽��ϳ��������
1~2���ӣ�����ӡ��Ϣ�е�rdcֵ����������*/
#define BATRDC				100
/*��·��ѹ�����еĵ�ص�ѹ�Ļ���*/
#define AXP_VOL_MAX			1
/*
	��繦��ʹ�ܣ�
        AXP22:0-�رգ�1-��
*/
#define CHGEN       1

/*
	���������ã�uA��0Ϊ�رգ�
		AXP22:300~2550,100/step
*/
/*������������uA*/
#define STACHGCUR			600*1000
/*������������uA*/
#define EARCHGCUR			1950*1000
/*���߳�������uA*/
#define SUSCHGCUR			1950*1000
/*�ػ���������uA*/
#define CLSCHGCUR			1950*1000

/*Ŀ�����ѹ��mV*/
/*
	AXP22:4100/4220/4200/4240
*/
#define CHGVOL				4200 * 1000
/*������С�����õ�����ENDCHGRATE%ʱ��ֹͣ��磬%*/
/*
	AXP22:10\15
*/ 
#define ENDCHGRATE			10
/*�ػ���ѹ��mV*/
/*
	ϵͳ��ƵĹػ�����ĵ�ض˵�ѹ����Ҫ��ػ��ٷֱȡ�
	��·��ѹ��Ӧ�ٷֱȱ��͵羯���ѹ�໥��ϲŻ�������
*/
//#define SHUTDOWNVOL			3300
#define SHUTDOWNVOL			3550 

/*adc���������ã�Hz*/
/*
	AXP22:100\200\400\800
*/
#define ADCFREQ				100
/*Ԥ��糬ʱʱ�䣬min*/
/*
	AXP22:40\50\60\70
*/
#define CHGPRETIME			50
/*������糬ʱʱ�䣬min*/
/*
	AXP22:360\480\600\720
*/
#define CHGCSTTIME			480


/*pek����ʱ�䣬ms*/
/*
	��power��Ӳ������ʱ�䣺
		AXP22:128/1000/2000/3000
*/
#define PEKOPEN				1000
/*pek����ʱ�䣬ms*/
/*
	��power���������жϵ�ʱ�䣬���ڴ�ʱ���Ƕ̰������̰���irq��
	���ڴ�ʱ���ǳ�������������irq��
		AXP22:1000/1500/2000/2500
*/
#define PEKLONG				1500
/*pek�����ػ�ʹ��*/
/*
	��power�������ػ�ʱ��Ӳ���ػ�����ʹ�ܣ�
		AXP22:0-���أ�1-�ػ�
*/
#define PEKOFFEN			1
/*pek�����ػ�ʹ�ܺ󿪻�ѡ��*/
/*
	��power�������ػ�ʱ��Ӳ���ػ���������ѡ��:
		AXP22:0-ֻ�ػ���������1-�ػ�������
*/
#define PEKOFFRESTART			0
/*pekpwr�ӳ�ʱ�䣬ms*/
/*
	������powerok�źŵ��ӳ�ʱ�䣺
		AXP20:8/16/32/64
*/
#define PEKDELAY			32
/*pek�����ػ�ʱ�䣬ms*/
/*
	��power���Ĺػ�ʱ����
		AXP22:4000/6000/8000/10000
*/
#define PEKOFF				6000
/*���¹ػ�ʹ��*/
/*
	AXP�ڲ��¶ȹ���Ӳ���ػ�����ʹ�ܣ�
		AXP22:0-���أ�1-�ػ�
*/
#define OTPOFFEN			0
/* ����ѹ����ʹ��*/
/*
	AXP22:0-�رգ�1-��
*/
#define USBVOLLIMEN		0
/*  �����ѹ��mV��0Ϊ������*/
/*
	AXP22:4000~4700��100/step
*/
#define USBVOLLIM			4000
/*  USB�����ѹ��mV��0Ϊ������*/
/*
	AXP22:4000~4700��100/step
*/
#define USBVOLLIMPC			4000

/* ����������ʹ��*/
/*
	AXP22:0-�رգ�1-��
*/
#define USBCURLIMEN		1
/* ���������mA��0Ϊ������*/
/*
	AXP22:500/900
*/
#define USBCURLIM			0
/* usb ���������mA��0Ϊ������*/
/*
	AXP22:500/900
*/
#define USBCURLIMPC			500
/* PMU �жϴ�������ʹ��*/
/*
	AXP22:0-�����ѣ�1-����
*/
#define IRQWAKEUP			0
/* N_VBUSEN PIN ���ܿ���*/
/*
	AXP22:0-���������OTG��ѹģ�飬1-���룬����VBUSͨ·
*/
#define VBUSEN			1
/* ACIN/VBUS In-short ��������*/
/*
	AXP22:0-AC VBUS�ֿ���1-ʹ��VBUS��AC,�޵���AC
*/
#define VBUSACINSHORT			0
/* CHGLED �ܽſ�������*/
/*
	AXP22:0-������1-�ɳ�繦�ܿ���
*/
#define CHGLEDFUN			1
/* CHGLED LED ��������*/
/*
	AXP22:0-���ʱled������1-���ʱled��˸
*/
#define CHGLEDTYPE			0
/* ���������У��ʹ��*/
/*
	AXP22:0-�رգ�1-��
*/
#define BATCAPCORRENT			0
/* �����ɺ󣬳�����ʹ��*/
/*
	AXP22:0-�رգ�1-��
*/
#define BATREGUEN			0
/* ��ؼ�⹦��ʹ��*/
/*
	AXP22:0-�رգ�1-��
*/
#define BATDET		1
/* PMU����ʹ��*/
/*
	AXP22:0-�رգ�1-�򿪰���Դ��16������PMU����
*/
#define PMURESET		0
/*�͵羯���ѹ1��%*/
/*
	����ϵͳ���������
	AXP22:5%~20%
*/
#define BATLOWLV1    15
/*�͵羯���ѹ2��%*/
/*
	����ϵͳ���������
	AXP22:0%~15%
*/
#define BATLOWLV2    5

#define ABS(x)				((x) >0 ? (x) : -(x) )

#ifdef	CONFIG_KP_AXP22
/*AXP GPIO start NUM,����ƽ̨ʵ���������*/
#define AXP22_NR_BASE 1024

/*AXP GPIO NUM,�������������LCD power�Լ�VBUS driver pin*/
#define AXP22_NR 5

/*��ʼ����·��ѹ��Ӧ�ٷֱȱ�*/
/*
	����ʹ��Ĭ��ֵ��������ø���ʵ�ʲ��Եĵ����ȷ��ÿ��
	��Ӧ��ʣ��ٷֱȣ��ر���Ҫע�⣬�ػ���ѹSHUTDOWNVOL�͵��
	������ʼУ׼ʣ�������ٷֱ�BATCAPCORRATE��������׼ȷ��
	AXP22����
*/
#define OCVREG0				0		 //3.13V
#define OCVREG1				0		 //3.27V
#define OCVREG2				0		 //3.34V
#define OCVREG3				0		 //3.41V
#define OCVREG4				0		 //3.48V
#define OCVREG5				0		 //3.52V
#define OCVREG6				0		 //3.55V
#define OCVREG7				2		 //3.57V
#define OCVREG8				4		 //3.59V
#define OCVREG9				7		 //3.61V
#define OCVREGA				10		 //3.63V
#define OCVREGB				11		 //3.64V
#define OCVREGC				14		 //3.66V
#define OCVREGD				19		 //3.7V
#define OCVREGE				24		 //3.73V 
#define OCVREGF				32		 //3.77V
#define OCVREG10		 	34                //3.78V
#define OCVREG11		 	41                //3.8V
#define OCVREG12		 	48                //3.82V 
#define OCVREG13		 	53                //3.84V
#define OCVREG14		 	55                //3.85V
#define OCVREG15		 	59                //3.87V
#define OCVREG16		 	65                //3.91V
#define OCVREG17		 	69                //3.94V
#define OCVREG18		 	74                //3.98V
#define OCVREG19		 	77                //4.01V
#define OCVREG1A		 	82                //4.05V
#define OCVREG1B		 	85                //4.08V
#define OCVREG1C		 	87                //4.1V 
#define OCVREG1D		 	89                //4.12V
#define OCVREG1E		 	91                //4.14V
#define OCVREG1F		 	100                //4.15V

/*  AXP�ж�*/
#define AXP_IRQ_USBLO		AXP22_IRQ_USBLO	//usb �͵�
#define AXP_IRQ_USBRE		AXP22_IRQ_USBRE	//usb �γ�
#define AXP_IRQ_USBIN		AXP22_IRQ_USBIN	//usb ����
#define AXP_IRQ_USBOV		AXP22_IRQ_USBOV	//usb ��ѹ
#define AXP_IRQ_ACRE			AXP22_IRQ_ACRE	//ac  �γ�
#define AXP_IRQ_ACIN			AXP22_IRQ_ACIN	//ac  ����
#define AXP_IRQ_ACOV		AXP22_IRQ_ACOV //ac  ��ѹ
#define AXP_IRQ_TEMLO		AXP22_IRQ_TEMLO //��ص���
#define AXP_IRQ_TEMOV		AXP22_IRQ_TEMOV //��ع���
#define AXP_IRQ_CHAOV		AXP22_IRQ_CHAOV //��س�����
#define AXP_IRQ_CHAST		AXP22_IRQ_CHAST //��س�翪ʼ
#define AXP_IRQ_BATATOU	AXP22_IRQ_BATATOU //����˳�����ģʽ
#define AXP_IRQ_BATATIN		AXP22_IRQ_BATATIN //��ؽ��뼤��ģʽ
#define AXP_IRQ_BATRE		AXP22_IRQ_BATRE //��ذγ�
#define AXP_IRQ_BATIN		AXP22_IRQ_BATIN //��ز���
#define AXP_IRQ_PEKLO		AXP22_IRQ_POKLO //power������
#define AXP_IRQ_PEKSH		AXP22_IRQ_POKSH //power���̰�

#define AXP_IRQ_CHACURLO	AXP22_IRQ_CHACURLO //������С������ֵ
#define AXP_IRQ_ICTEMOV		AXP22_IRQ_ICTEMOV //AXPоƬ�ڲ�����
#define AXP_IRQ_EXTLOWARN2	AXP22_IRQ_EXTLOWARN2 //APS��ѹ�����ѹ2
#define AXP_IRQ_EXTLOWARN1	AXP22_IRQ_EXTLOWARN1 //APS��ѹ�����ѹ1

#define AXP_IRQ_GPIO0TG		AXP22_IRQ_GPIO0TG //GPIO0������ش���
#define AXP_IRQ_GPIO1TG		AXP22_IRQ_GPIO1TG //GPIO1������ش���

#define AXP_IRQ_PEKFE		AXP22_IRQ_PEKFE //power���½��ش���
#define AXP_IRQ_PEKRE		AXP22_IRQ_PEKRE //power�������ش���
#define AXP_IRQ_TIMER		AXP22_IRQ_TIMER //��ʱ����ʱ

#endif

/*ѡ����Ҫ�򿪵��ж�ʹ��*/
static const uint64_t AXP22_NOTIFIER_ON = (AXP_IRQ_USBIN |AXP_IRQ_USBRE |
				       		                            AXP_IRQ_ACIN |AXP_IRQ_ACRE |
				       		                            AXP_IRQ_BATIN |AXP_IRQ_BATRE |
				       		                            AXP_IRQ_CHAST |AXP_IRQ_CHAOV|
						                            (uint64_t)AXP_IRQ_PEKFE);

/* ��Ҫ�������ţ��usb�ػ�������bootʱpower_start����Ϊ1������Ϊ0*/
#define POWER_START 1


#endif
