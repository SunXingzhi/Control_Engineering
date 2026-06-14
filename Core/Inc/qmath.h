#ifndef QMATH_H
#define QMATH_H

// 快速正弦函数
float qsin_rad(float x);

// 快速余弦函数（弧度参数）
float qcos_rad(float x) ;

// 快速正切函数
float qtan_rad(float x) ;

// 快速反正弦函数
float qasin_rad(float x);

// 快速反余弦函数
float qacos_rad(float x) ;

// 快速反正切函数
float qatan_rad(float x);

// 快速反正切函数2
float qatan2_rad(float y, float x);


void QMATH_test();

#endif