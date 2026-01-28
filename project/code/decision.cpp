#include "decision.hpp"

/**************角点识别部分*/
PT_Judge_TypeDef Lline_pt = LOST_PT;  // 左角点标志位 0-丢失 1-直道 2-弯道 3-L角点 4-Y角点
PT_Judge_TypeDef Rline_pt = LOST_PT;  // 右角点标志位 0-丢失 1-直道 2-弯道 3-L角点 4-Y角点
PT_Identify_Typedef Lline_pt_idf = {S_PT,S_PT,1};    /* 左边线角点识别 */
PT_Identify_Typedef Rline_pt_idf = {S_PT,S_PT,1};    /* 右边线角点识别 */
/*角点识别部分**************/

/********************状态机*/
ElementFlag_TypeDef normal_sta  =  {'n',0,normal_proc};                //正常行驶
ElementFlag_TypeDef cross_sta   =  {'c',0,cross_proc};                 //十字
ElementFlag_TypeDef Lcircle_sta =  {'L',begin_circle,Lcircle_proc};    //左圆环
ElementFlag_TypeDef Rcircle_sta =  {'R',begin_circle,Rcircle_proc};    //右圆环
ElementFlag_TypeDef *cur_sta    =  &normal_sta;                        //状态机指针默认正常行驶
/*状态机********************/

/******************其他变量*/
std::atomic<bool> circle_flag(0);      //圆环减速标志位
/*其他变量******************/

/*--------------------------功能函数----------------------------*/

// 一次图像处理
void image_proc(){   
    /*----------图像处理----------*/
    // 等待采样数据
    uvc.wait_image_refresh();
    img_gray = uvc.get_gray_image_ptr();

    start_thre = get_otsu_thres(img_gray,0,IMG_W,TRACK_HEIGHT_MAX,IMG_H);      // 二值化
    //状态机决策
    line_process(IMG_H,IMG_H/2);

}

/**
 * @brief 边线角点识别
 * @param out       输出值
 * @param nms_val   极大值
 * @param linelen   原始线长
 */
void PT_Identify_Typedef::pt_identify(PT_Judge_TypeDef &out, float nms_val, int linelen){
    //先判断是否丢线
    if (linelen < LINE_LOST_LENGTH){
        pt = LOST_PT;
    }
    else{
        //直道判断
        if (nms_val <= STRAIGHT_UP) {
            //无效值
            if (linelen < LINE_ST_MIN_LEN)
                pt = INVAL_PT; 
            //直道
            else
                pt = S_PT;
        }
        //弯道
        else if (nms_val <= LPT_LOW){
            pt = T_PT;
        }
        //L角点
        else if (nms_val <= LPT_UP){
            pt = L_PT;
        }
        //Y角点
        else if (nms_val > LPT_UP){
            pt = Y_PT;
        }
    }

    //稳定时赋值 
    if (pt_cnt >= __JUDGE_CNT) {
        out = pt;
        pt_cnt = 1;
    }
    else {
        //数值有变化
        if (pt != per_pt) pt_cnt = 1;   //从1开始 1也算一次
        else pt_cnt ++;
    }
    per_pt = pt;
}

/**
 * @brief 边线角点识别
 */
void lines_ptIdentify(void){
    /*--左边线--*/
    Lline_pt_idf.pt_identify(Lline_pt, nms_Lline, sampled_Lline_num);

    /*--右边线--*/
    Rline_pt_idf.pt_identify(Rline_pt, nms_Rline, sampled_Rline_num);
}

/**
 * @brief 补左边线
 */
void Supplement_Lline(void){
    supplement_line(sampled_Lline, &sampled_Lline_num, nms_Lline_idx, sampled_dist * M2PIX);
}

/**
 * @brief 补右边线
 */
void Supplement_Rline(void){
    supplement_line(sampled_Rline, &sampled_Rline_num, nms_Rline_idx, sampled_dist * M2PIX);
}

/**
 * @brief 巡右边线
 * 状态机输出结果赋值 
 */
void Track_Rline(void){
    Mline = R2Mline;
    aim_point.flag = 1;
}

/**
 * @brief 巡左边线
 * 状态机输出结果赋值 
 */
void Track_Lline(void){
    Mline = L2Mline;
    aim_point.flag = 1;
}

/**
 * @brief 给定角度 不巡线
 * @param __angle 给定的角度
 */
void Force_Angle(float __angle){
    Mline = nullptr;
    aim_point.angle = __angle;
    aim_point.flag = 0;
}

/**
 * @brief 将状态机切换为正常巡线
 *        切换时按照线长巡线
 */
void Cur_Sta_SetNormal(void){
    cur_sta = &normal_sta;
    circle_flag.store(0);

    //根据线长巡线
    if (Lline_num >= Rline_num)
        Track_Lline();
    else 
        Track_Rline();
    //打印信息
    std::cout << "当前状态为：正常巡线" << std::endl;
}

/**
 * @brief 将状态机切换为十字处理
 *        切换时优先补左边线
 */
void Cur_Sta_SetCross(void){
    cur_sta = &cross_sta;
    //优先补左边线
    Supplement_Lline();
    //巡左边线
    Track_Lline();
    std::cout << "当前状态为：十字处理" << std::endl;
}

/**
 * @brief 将状态切换为右圆环处理
 */
void Cur_Sta_SetRcircle(void){
    cur_sta = &Rcircle_sta;
    //重置
    Rcircle_sta.flag = init_circle;
    circle_flag.store(1);   //进入圆环减速
    std::cout << "当前状态为：右圆环" << std::endl;
}

/**
 * @brief 将状态切换为左圆环处理
 */
void Cur_Ste_SetLcircle(void){
    cur_sta = &Lcircle_sta;
    //重置
    Lcircle_sta.flag = init_circle;
    circle_flag.store(1);   //进入圆环减速
    std::cout << "当前状态为：左圆环" << std::endl;
}

/**********************************状态机决策**********************************/

/**
 * @brief 状态机决策
 *        输入：左右边线极大角度、线长
 *        输出：Lline_pt Rline_pt -- 左右角点
 */
void sta_decision(void){
    /*--------状态机---------*/
    //初始状态为不巡线 即保持上次状态
    aim_point.flag = 0;
    cur_sta->func();
    /*确定最终巡线*/
    std::cout << "是否进入寻线：" << aim_point.flag << std::endl;
    if (aim_point.flag){
        std ::cout << "巡线选择："<<std::endl;
        //不巡或无效或丢失的线
        if (Mline == L2Mline && (Lline_pt & TRABLE_PT)) {
            Mline_num = sampled_Lline_num;
            track_leftline();
            std ::cout << "左边线" << std::endl;
        }
        else if (Mline == R2Mline && (Rline_pt & TRABLE_PT)) {
            Mline_num = sampled_Rline_num;
            track_rightline();
            std ::cout << "右边线" << std::endl;
        }
        else {
            //不巡线
            aim_point.flag = 0;
            std::cout << "不巡线" << std::endl;
        }
    }
}

/**
 * @brief 正常行驶状态机处理 
 */
void normal_proc(void){
    // // 边线处理
    line_process(IMG_H,IMG_H/2);
    // //角点识别

    lines_ptIdentify();
    //预瞄点赋值
    aim_point.idx = AIM_IDX;

    /*正常根据线长判断巡线*/
    //左边丢线 右边普通角点
    if ((Lline_pt & INTRABLE_PT) && (Rline_pt & NORMAL_PT)){
        //巡右边线
        Track_Rline();
    }
    //左边普通角点 右边丢线
    else if ((Lline_pt & NORMAL_PT) && (Rline_pt & INTRABLE_PT)){
        //巡左边线
        Track_Lline();
    }
    //左右都是普通角点
    else if ((Lline_pt & NORMAL_PT) && (Rline_pt & NORMAL_PT) ){
        // Mline == L2Mline
        if (Mline == L2Mline){
            //右边较长 切换为右边
            if (Rline_num > Lline_num + __N_DFT_LEN_THRE)
                Track_Rline();
            else 
                Track_Lline();
        }
        // Mline == R2Mline || null
        else {
            //左边较长 切换为左边
            if (Lline_num > Rline_num + __N_DFT_LEN_THRE)
                Track_Lline();
            else 
                Track_Rline();
        }
    }
    /*发现十字*/
    //左/右边线存在L角点
    else if ((Lline_pt == L_PT) && (Rline_pt == L_PT))
    {
        Cur_Sta_SetCross();
    }
    /*发现右圆环*/
    else if (Rline_pt == Y_PT){
        if (Lline_pt & NORMAL_PT){
            //切换为右圆环
            // Cur_Sta_SetRcircle();
            //输出 巡左边线
            Track_Lline();    
        }
        //补右边线 防止另一边线没找到而直接冲入环岛
        else if (Lline_pt == LOST_PT){
            Supplement_Rline();
            
            //输出 巡右边线
            Track_Rline();
        }
        // else 其他状态无效
    }
    /*发现左圆环*/
    else if (Lline_pt == Y_PT){
        if (Rline_pt & NORMAL_PT){
            //切换为左圆环
            // Cur_Ste_SetLcircle();
            //输出 巡右边线
            Track_Rline();   
        }
        //补左边线 防止另一边线没找到而直接冲入环岛
        else if (Rline_pt == LOST_PT){
            Supplement_Lline();

            //输出 巡左边线
            Track_Lline();
        }
        // else 其他状态无效
    }
    // else 其他状态无效
}

/**
 * @brief 十字处理
 */
void cross_proc(void){
    // 边线处理 不向上寻找
    line_process(IMG_H,IMG_H * 3/4);
    //角点识别
    lines_ptIdentify();

    //预瞄点
    aim_point.idx = AIM_IDX;

    //驶出十字：左右均为普通边线
    if ((Lline_pt & NORMAL_PT) 
    &&  (Rline_pt & NORMAL_PT))
    {
        Cur_Sta_SetNormal();
    }
    // 未丢线则优先巡左边线
    else if (Lline_pt & SUPPABLE_PT){
        //补左边线
        Supplement_Lline();
        //巡左边线
        Track_Lline();
    }
    else if (Rline_pt & SUPPABLE_PT){
        //补右边线
        Supplement_Rline();
        //巡右边线
        Track_Rline();
    }
    // else 其他状态无效
}

/**
 * @brief 左圆环处理
 */
void Lcircle_proc(void){
    //静态变量
    static bool begin_find_left;
    static bool found_L_Y_PT;

    //参数初始化
    if (cur_sta->flag == init_circle)
    {
        begin_find_left = 0;
        found_L_Y_PT = 0;

        cur_sta->flag = begin_circle;
        std::cout << "环岛状态 begin_circle" << std::endl;
    }

    //防误判 左边线为直道
    if (
       cur_sta->flag == begin_circle
    && (Lline_pt == S_PT && sampled_Lline_num > 30)
    )
    {
        cur_sta = &normal_sta;
        circle_flag.store(0);
    }
    //准备驶入环岛
    else if (cur_sta->flag == begin_circle){
        //发现右边线则巡右边线
        if (Rline_pt != LOST_PT){
            Mline = R2Mline;
            aim_point.idx = AIM_IDX;
        }
        //若右边丢失，左边仍有Y角点则补线，防止冲入环岛
        else if (Lline_pt == Y_PT){
            supplement_line(sampled_Lline, &sampled_Lline_num, nms_Lline_idx, sampled_dist * M2PIX);
            Mline = L2Mline;
            aim_point.idx = AIM_IDX;
        }
        //意外情况
        else {
           aim_point.flag = 0;
           aim_point.angle = 0;
        }

        //等待左边内环岛边线
        if (Lline_pt == T_PT && sampled_Lline_num > 15){
            //进入下一个状态
            cur_sta->flag = into_circle;
            //巡右边线 中预瞄点
            Mline = R2Mline;
            aim_point.idx = AIM_IDX;
        }
    }
    //驶入环岛
    else if (cur_sta->flag == into_circle)
    {
        //此时巡右边线 中预瞄点
        Mline = R2Mline;
        aim_point.idx = AIM_IDX;

        //寻找合适进环岛时机
        if (Lline_pt == T_PT && sampled_Lline_num >= 15){
            begin_find_left = 1;
            //进环岛巡左边线
            Mline = L2Mline;
            aim_point.idx = AIM_IDX;
        }
        //发现左边线后则不再巡右边线
        else if (begin_find_left){
            aim_point.angle = -32;
            //不巡线
            aim_point.flag = 0;
        }

        //根据线长来判断是否进入圆环
        if (sampled_Rline_num < 30 && Rline_pt == T_PT){
            //进入下一个状态
            cur_sta->flag = in_circle;
            //巡右边线 中预瞄点
            Mline = R2Mline;
            aim_point.idx = AIM_IDX;
        }
    }
    //环岛内巡线
    else if (cur_sta->flag == in_circle){
        //巡右边线 中预瞄点
        Mline = R2Mline;
        aim_point.idx = AIM_IDX;

        //右边线丢失
        if (!Rline_pt){
            aim_point.angle = -35;
            //不巡线
            aim_point.flag = 0;
        }
        //状态机判断 等待右边Y角点
        if (Rline_pt == Y_PT || Rline_pt==S_PT ){
            //补右边线
            Supplement_Rline();
            //进入下一个状态
            cur_sta->flag = out_circle;
            //继续巡右边线
        }
    }
    //驶出环岛状态
    else if (cur_sta->flag == out_circle){
        //继续巡右边线
        Mline = R2Mline;
        aim_point.idx = AIM_IDX;

        //查看右边线是否还有Y角点
        if (Rline_pt == Y_PT){
            //补右边线
            Supplement_Rline();
            aim_point.idx = AIM_IDX;
        }
        else if (Rline_pt == S_PT) {
            //中预瞄点
            aim_point.idx = AIM_IDX;
            //更新状态
            cur_sta->flag = outof_circle;
            //继续巡右边线
        }
        //否则发现左边线则往右偏
        else if (Lline_pt != LOST_PT){
            aim_point.flag = 0;
            aim_point.angle = 5;
        }
        //不巡线
        else {
            aim_point.flag = 0;
            aim_point.angle = -45;
        }
    }
    //判断驶出环岛
    else if (cur_sta->flag == outof_circle){
        //继续巡右边线 中预瞄点
        Mline = R2Mline;
        aim_point.idx = AIM_IDX;

        //等待左边线边线出现Y角点 再消失
        //Y角点消失 （先判断是否消失）
        if (found_L_Y_PT){
            //角点消失
            if (Lline_pt < L_PT && Lline_pt != LOST_PT){
                //驶出环岛
                cur_sta = &normal_sta;
                //标志位清零
                circle_flag.store(0);    //减速标志位
            }
        }
        //出现Y或L角点
        if (Lline_pt >= L_PT){
            found_L_Y_PT = 1;
        }
    }
}

/**
 * @brief 右圆环处理
 */
void Rcircle_proc(void){
    //静态变量
    static uint8 __begin_lose_l = 0;     //驶入圆环阶段 左边丢失标志位
    static bool __outof_found_RY = 0;    //离开圆环阶段 发现右边Y角点标志位

    //参数初始化
    if (cur_sta->flag == init_circle)
    {
        __begin_lose_l = 0;
        __outof_found_RY = 0;

        cur_sta->flag = begin_circle;
        std::cout << "圆环状态:begin_circle" << std::endl;
    }

    //准备驶入环岛
    if (cur_sta->flag == begin_circle)
    {
        // 边线处理
        line_process(IMG_H,IMG_H/2);
        //角点识别
        lines_ptIdentify();

        //左边为普通边线 右边不为弯道
        if ((Lline_pt & NORMAL_PT) && Rline_pt != T_PT){
            //巡左边线
            Track_Lline();
        }    
        // 左边任意 右边为弯道
        else if (Rline_pt == T_PT){
            //下一个状态
            cur_sta->flag = into_circle;
            //巡右边线
            Track_Rline();
        }
        // 左边丢失 右边有Y角点
        else if (Lline_pt == LOST_PT && Rline_pt == Y_PT){
            //补右边线 并巡右边线
            Supplement_Rline();
            Track_Rline();
        }
        // 右边为直道 防止误判
        else if (Rline_pt == S_PT){
            Cur_Sta_SetNormal();
        }
        // else 其他状态无效
    }
    //驶入环岛
    else if (cur_sta->flag == into_circle)
    {
        // 未丢线
        if (__begin_lose_l < __RC_BEGIN_LOSE_THRE){
            // 边线处理
            line_process(IMG_H,IMG_H*3/4);
            //角点识别
            lines_ptIdentify();

            // 左边丢失 右边任意
            if (Lline_pt == LOST_PT){
                __begin_lose_l ++;
                //右边为普通角点
                if (Rline_pt & NORMAL_PT){
                    Track_Rline();
                }
                // else 其他状态无效
            }
            // 左边线未丢失
            else {
                __begin_lose_l = 0; //计数清零
                //右边为普通角点
                if (Rline_pt & NORMAL_PT){
                    Track_Rline();
                }
                // else 其他状态无效
            }
        }
        //丢线
        else{
            // 边线处理
            line_process(IMG_H,TRACK_HEIGHT_MAX);
            //角点识别
            lines_ptIdentify();

            //左边为弯道 右边任意
            if (Lline_pt == T_PT){
                //下一个状态
                cur_sta->flag = in_circle;
                __begin_lose_l = 0; //标志位清空
                Track_Lline();
            }
            //左边非弯道 右边为普通角点
            else if (Lline_pt != T_PT && (Rline_pt & NORMAL_PT)){
                Track_Rline();
            }
            //else 其他状态无效
        }
    }
    //环岛内巡线
    else if (cur_sta->flag == in_circle)
    {
        // 边线处理
        line_process(IMG_H,IMG_H/2);
        //角点识别
        lines_ptIdentify();

        // 左边弯道 右边任意
        if (Lline_pt == T_PT){
            //巡左边线
            Track_Lline();
        }
        //左边Y角点 右边任意
        else if (Lline_pt == Y_PT){
            //下一个状态
            cur_sta->flag = out_circle;
            //补左边线
            Supplement_Lline();
            //巡左边线
            Track_Lline();
        }
        // else 其他无效
    }
    //驶出环岛状态
    else if (cur_sta->flag == out_circle)
    {
        // 边线处理
        line_process(IMG_H,TRACK_HEIGHT_MAX);
        //角点识别
        lines_ptIdentify();

        // 左边Y角点 右边任意
        if (Lline_pt == Y_PT){
            // 补左边线
            Supplement_Lline();
            // 巡左边线
            Track_Lline();
        }
        // 左边普通边线 右边任意
        else if (Lline_pt & NORMAL_PT){
            // 下一个状态
            cur_sta->flag = outof_circle;
            // 巡左边线
            Track_Lline();
        }
        // 左边丢失 右边任意
        else if (Lline_pt == LOST_PT){
            // 给固定角度
            Force_Angle(__RC_OUT_FIXED_ANGLE);
        }
        // else 其他状态无效
    }
    //判断驶出环岛
    else if (cur_sta->flag == outof_circle)
    {
        // 边线处理
        line_process(IMG_H,TRACK_HEIGHT_MAX);
        //角点识别
        lines_ptIdentify();

        // 左边普通角点 右边Y角点
        if ((Lline_pt & NORMAL_PT) && Rline_pt == Y_PT){
            // 发现Y角点标志位置1
            __outof_found_RY = 1;
            // 巡左边线
            Track_Lline();
        }
        // 左边普通角点 右边普通角点 Y角点标志位为1
        else if ((Lline_pt & NORMAL_PT) && (Rline_pt & NORMAL_PT) && __outof_found_RY){
            // 标志位清空
            __outof_found_RY = 0;
            // 切换为普通巡线模式
            Cur_Sta_SetNormal();
        }
        // else 其他状态无效
    }
    // 错误
    else {
        Cur_Sta_SetNormal();
    }
}

/*************************外部接口*************************/

/**
 * @brief 状态机复位
 */
void decision_sta_reset(void){
    cur_sta = &normal_sta;
}

/**
 * @brief 获取左边角点 
 */
char decision_get_llpt(void){
    switch (Lline_pt)
    {
    case LOST_PT:
        return 'l';
        break;
    case S_PT:
        return 's';
        break;
    case T_PT:
        return 't';
        break;
    case L_PT:
        return 'L';
        break;
    case Y_PT:
        return 'Y';
        break;
    case INVAL_PT:
        return 'i';
        break;
    default:
        return '*';
        break;
    }
}

/**
 * @brief 获取右边角点 
 */
char decision_get_rlpt(void){
    switch (Rline_pt)
    {
    case LOST_PT:
        return 'l';
        break;
    case S_PT:
        return 's';
        break;
    case T_PT:
        return 't';
        break;
    case L_PT:
        return 'L';
        break;
    case Y_PT:
        return 'Y';
        break;
    case INVAL_PT:
        return 'i';
        break;
    default:
        return '*';
        break;
    }
}
