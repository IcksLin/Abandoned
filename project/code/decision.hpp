#ifndef __DECISION_HPP__
#define __DECISION_HPP__

#include "zf_common_typedef.hpp"
#include "imgproc.hpp"

extern std::atomic<bool> circle_flag;

void image_proc();

void sta_decision(void);
void normal_proc(void);
void cross_proc(void);
void Lcircle_proc(void);
void Rcircle_proc(void);
void decision_sta_reset(void);
char decision_get_rlpt(void);
char decision_get_llpt(void);
void decision_sta_reset(void);

#endif // __DECISION_HPP__