#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

// ��ʼ��ƽ�����ϵͳ
void control_init();

// �����˱��
void control_changelp(float new_lp);

// ƽ����ƵĽ��̺���
void CONTROL_proc();

// ��λ
void control_reset();


float get_omega_ref();

uint64_t get_last_timeus();

void DWT_Init(void);

uint32_t DWT_GetTick_us(void);


#endif
