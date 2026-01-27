#include "imgproc.hpp"
using namespace cv;

/*决策变量******************/

/******************图像变量*/
//图像
Mat frame_color;                // 用于处理的图像帧
Mat frame_gray;                 // 灰度图像帧
Mat frame_bin;                  // 二值图像帧

uint8_t* img_gray;              // 灰度图像指针
AimPoint_TypeDef aim_point; 


/*图像变量******************/

/**************边线处理变量*/
/*参数变量*/
uint8_t all_block_size = 7;
uint8_t adapt_clip = 7;         // 自适应迷宫巡线偏差
uint8_t start_thre = 130;
float avg_angle;
float map_x[IMG_H][IMG_W];      // 原图点(x,y)透视后x坐标
float map_y[IMG_H][IMG_W];      // 原图点(x,y)透视后y坐标
// 方向数组
int dir_front[4][2]      = {{0,  -1},{1, 0},{0, 1},{-1, 0}};
int dir_frontleft[4][2]  = {{-1, -1},{1,-1},{1, 1},{-1, 1}};
int dir_frontright[4][2] = {{1,  -1},{1, 1},{-1,1},{-1,-1}};

/*边线变量*/
//采样前后边线长度
int Lline_num, Rline_num;
int sampled_Lline_num, sampled_Rline_num;
int Mline_num;
// 边线
int Lline[POINTS_MAX_LEN][2], Rline[POINTS_MAX_LEN][2];                      // 原始
int ud_Lline[POINTS_MAX_LEN][2], ud_Rline[POINTS_MAX_LEN][2];                // 去畸变
float per_Lline[POINTS_MAX_LEN][2], per_Rline[POINTS_MAX_LEN][2];            // 透视
float blurred_Lline[POINTS_MAX_LEN][2], blurred_Rline[POINTS_MAX_LEN][2];    // 滤波
float sampled_Lline[POINTS_MAX_LEN][2], sampled_Rline[POINTS_MAX_LEN][2];    // 等距采样
float L2Mline[POINTS_MAX_LEN][2], R2Mline[POINTS_MAX_LEN][2];                // 左右得到的中线
float (*Mline)[2] = L2Mline;                                                 // 最终中线
float dangle_Lline[POINTS_MAX_LEN], dangle_Rline[POINTS_MAX_LEN];            // 局部角度变化率
// 非极大抑制
float nms_Lline,nms_Rline;          // 角点值
int nms_Lline_idx,nms_Rline_idx;    // 索引

/*透视矩阵*/
cv::Mat M = (cv::Mat_<float>(3, 3) <<
    -0.8396682791983416,-2.5742916378714584,149.2104353835522,
    0.0,-4.346924671734624,188.4882515549413,
    -0.0,-0.03178991015894955,1.0);

cv::Mat M_Reverse = (cv::Mat_<float>(3, 3) <<
    -1.1909465020576129,1.5702850175278156,-118.2786313062034,
    -0.0,0.6078658160413988,-114.57556484566084,
    -0.0,0.019323999680652622,-2.6423469128544568);
/*边线处理变量**************/



// 完整的一个边线处理
void line_process(uint8_t height_start, uint8_t height_min){

    // 左右巡线
    search_Lline(height_start, height_min);
    search_Rline(height_start, height_min);

    // 巡到的线小于LINE_LOST_LENGTH个则判断为丢线 不进行后续处理
    //左边线
    if (Lline_num <= LINE_LOST_LENGTH)
        sampled_Lline_num = 0;  //表示丢线
    else{
        // 点集透视
        perspective_transform_points(Lline,Lline_num,per_Lline);
        // 点集滤波
        blur_points(per_Lline,Lline_num,blurred_Lline);
        // 点集采样
        resample_points(blurred_Lline,Lline_num,sampled_Lline,&sampled_Lline_num);
        // 点集角度
        local_angle_points(sampled_Lline,sampled_Lline_num,dangle_Lline);
        // 极大角度
        nms_angle(dangle_Lline,sampled_Lline_num,&nms_Lline,&nms_Lline_idx);
    }

    //右边线
    if (Rline_num <= LINE_LOST_LENGTH)
        sampled_Rline_num = 0;  //表示丢线
    else{
        perspective_transform_points(Rline,Rline_num,per_Rline);
        blur_points(per_Rline,Rline_num,blurred_Rline);
        resample_points(blurred_Rline,Rline_num,sampled_Rline,&sampled_Rline_num);
        local_angle_points(sampled_Rline,sampled_Rline_num,dangle_Rline);
        nms_angle(dangle_Rline,sampled_Rline_num,&nms_Rline,&nms_Rline_idx);
    }
}

//大津法二值化 返回阈值
uint8 get_otsu_thres(uint8 *img, int x0, int x1, int y0, int y1){
    /*灰度直方图参数*/
    uint16 histogram[256] = {0}; // 灰度直方图
    uint32 min_value, max_value;

    uint32 pix_amount = 0;   // 像素点总数
    uint32 pix_integral = 0; // 灰度值总数

    uint32 pix_back_amount = 0;   // 前景像素点总数
    uint32 pix_back_integral = 0; // 前景灰度值

    int32 pix_fore_amount = 0;   // 背景像素点总数
    int32 pix_fore_integral = 0; // 背景灰度值

    float omega_back, omega_fore, micro_back, micro_fore, sigma_beta, sigma; // 类间方差：浮点型更精确

    uint8 thres_result = 0;

    // 隔一行取一个值，更快
    for (int y = y0; y < y1; y +=2)
        for (int x = x0; x < x1; x +=2)
            histogram[IMG_AT(img, x, y)]++;

    for (min_value = 0; min_value < 256 && histogram[min_value] == 0; min_value++)
    {
        ; // 获取最小灰度的值
    }
    for (max_value = 255; max_value > min_value && histogram[min_value] == 0; max_value--)
    {
        ; // 获取最大灰度的值
    }

    if (max_value == min_value)
    {
        return ((uint8)(max_value)); // 图像中只有一个颜色
    }
    if (min_value + 1 == max_value)
    {
        return ((uint8)(min_value)); // 图像中只有二个颜色
    }

    /*OSTU大律法*/
    pix_integral = 0;
    for (uint16 j = (uint16)min_value; j <= max_value; j++)
    {
        pix_amount += histogram[j];       // 像素总数
        pix_integral += histogram[j] * j; // 灰度值总数
    }
    sigma_beta = -1;

    for (uint16 j = (uint16)min_value; j < max_value; j++)
    {
        pix_back_amount = pix_back_amount + histogram[j];                                        // 前景像素点数
        pix_fore_amount = pix_amount - pix_back_amount;                                          // 背景像素点数
        omega_back = (float)pix_back_amount / pix_amount;                                        // 前景像素百分比
        omega_fore = (float)pix_fore_amount / pix_amount;                                        // 背景像素百分比
        pix_back_integral += histogram[j] * j;                                                   // 前景灰度值
        pix_fore_integral = pix_integral - pix_back_integral;                                    // 背景灰度值
        micro_back = (float)pix_back_integral / pix_back_amount;                                 // 前景灰度百分比
        micro_fore = (float)pix_fore_integral / pix_fore_amount;                                 // 背景灰度百分比
        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore); // 计算类间方差
        if (sigma > sigma_beta)                                                                  // 遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            sigma_beta = sigma;
            thres_result = (uint8)j;
        }
    }
    return thres_result; // 返回最佳阈值;
}

// 保存映射
void save_per_map(void) {
    // 创建写入文本对象 - 明确使用std命名空间
    std::ofstream fout("maps/per_map.txt");
    
    // 检查文件是否成功打开
    if (!fout.is_open()) {
        std::cerr << "错误：无法打开文件 maps/per_map.txt" << std::endl;
        return;
    }
    
    // 计算透视映射
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {   
            float U = M.at<float>(0, 0) * x + M.at<float>(0, 1) * y + M.at<float>(0, 2);
            float V = M.at<float>(1, 0) * x + M.at<float>(1, 1) * y + M.at<float>(1, 2);
            float W = M.at<float>(2, 0) * x + M.at<float>(2, 1) * y + M.at<float>(2, 2);
            
            // 避免除以零
            if (W != 0.0f) {
                map_x[y][x] = U / W;
                map_y[y][x] = V / W;
            } else {
                map_x[y][x] = 0.0f;
                map_y[y][x] = 0.0f;
            }
        }
    }
    
    // 设置四位小数精度
    fout << std::fixed << std::setprecision(4);
    
    // 写入map_x
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            fout << map_x[y][x];
            if (x != IMG_W - 1) fout << " ";
        }
        fout << std::endl;
    }
    
    // 写入空行分隔
    fout << std::endl;
    
    // 写入map_y
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            fout << map_y[y][x];
            if (x != IMG_W - 1) fout << " ";
        }
        fout << std::endl;
    }
    
    // 关闭文件
    fout.close();
    
    // std::cout << "透视映射已保存到 maps/per_map.txt" << std::endl;
}

// /*******************************显示函数*********************************/
/**
 * @brief 对点应用透视矩阵
 */
void point_per(const cv::Mat& M, float x, float y, int& x_out, int& y_out) {
    const float M11 = M.at<float>(0, 0), M12 = M.at<float>(0, 1), M13 = M.at<float>(0, 2); 
    const float M21 = M.at<float>(1, 0), M22 = M.at<float>(1, 1), M23 = M.at<float>(1, 2); 
    const float M31 = M.at<float>(2, 0), M32 = M.at<float>(2, 1), M33 = M.at<float>(2, 2);

    float X = M11 * x + M12 * y + M13 * 1.0f;
    float Y = M21 * x + M22 * y + M23 * 1.0f;
    float W = M31 * x + M32 * y + M33 * 1.0f;
    x_out = (int)(X/W+0.5);
    y_out = (int)(Y/W+0.5);
}
/*---------------------------边线处理-----------------------------*/

// 透视变换
void perspective_transform_points(int pts_in[][2],int num,float pts_out[][2]){
    for (int i=0;i<num;i++){
        int x = pts_in[i][0];
        int y = pts_in[i][1];
        pts_out[i][0] = map_x[y][x];
        pts_out[i][1] = map_y[y][x];
    }
}

// 左手迷宫巡线
void findline_lefthand_adaptive(int x, int y){
    int half = all_block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < Lline_num && half < x && x < IMG_W - half - 1 && TRACK_HEIGHT_MAX < y && y < IMG_H - half - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += IMG_AT(img_gray, x + dx, y + dy);
            }
        }
        local_thres /= all_block_size * all_block_size;
        local_thres -= adapt_clip;

        uint8_t front_value = IMG_AT(img_gray, x + dir_front[dir][0], y + dir_front[dir][1]);
        uint8_t frontleft_value = IMG_AT(img_gray, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Lline[step][0] = x;
            Lline[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            Lline[step][0] = x;
            Lline[step][1] = y;
            step++;
            turn = 0;
        }
        frame_color.at<Vec3b>(y,x) = Vec3b(0,0,255);
    }
    Lline_num = step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(int x, int y){
    int half = all_block_size / 2;
    int step = 0, dir = 0, turn = 0;
    while (step < Rline_num && half < x && x < IMG_W - half - 1 && TRACK_HEIGHT_MAX < y && y < IMG_H - half - 1 && turn < 4) {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) {
            for (int dx = -half; dx <= half; dx++) {
                local_thres += IMG_AT(img_gray, x + dx, y + dy);
            }
        }
        local_thres /= all_block_size * all_block_size;
        local_thres -= adapt_clip;

        uint8_t front_value = IMG_AT(img_gray, x + dir_front[dir][0], y + dir_front[dir][1]);
        uint8_t frontright_value = IMG_AT(img_gray, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (frontright_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Rline[step][0] = x;
            Rline[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            Rline[step][0] = x;
            Rline[step][1] = y;
            step++;
            turn = 0;
        }
        frame_color.at<Vec3b>(y,x) = Vec3b(0,255,0);
    }
    Rline_num = step;
}

// 寻找左起始点并巡线
void search_Lline(uint8_t height_start, uint8_t height_min){
    uint8_t begin_x,begin_y;
    bool found_flag;

    /*寻找左边起始线*/
    begin_y = height_start - all_block_size/2 - 2;  //从指定起点开始向上
    begin_x = all_block_size/2 + 1;                 //最左边 要大于all_block_size/2
    found_flag = 0;

    //如果该起始点为边线外则（即黑点）按x轴向内寻找
    if (IMG_AT(img_gray, begin_x, begin_y) <= start_thre){
        //最多到中心
        for(;begin_x<IMG_W/2 - CAR_IMGAGE_W/2;begin_x++) {
            uint8_t i=0;
            //发现白点
            if (IMG_AT(img_gray, begin_x, begin_y) > start_thre){
                //向内再延伸check_dis个像素点 判断是否为噪点
                for (i=1;i<=CHECK_DIS;i++){
                    //为噪点
                    if (IMG_AT(img_gray, begin_x+i, begin_y) <= start_thre){
                        begin_x += i+1; //再移动对应个像素点后继续
                        break;
                    }
                }
            }
            //不是噪点
            if (i > CHECK_DIS){
                found_flag = 1;
                break;
            }
        }
    }
    //否则（即白点）按y轴
    else{
        for (;begin_y > height_min;begin_y--){
            uint8_t i=0;
            //发现黑点 则判断是否为噪点
            if (IMG_AT(img_gray, begin_x, begin_y) <= start_thre){
                //向上再延伸check_dis个像素点 判断是否为噪点
                for (i=1;i<=CHECK_DIS;i++){
                    //为噪点
                    if (IMG_AT(img_gray, begin_x, begin_y-i) > start_thre){
                        begin_y -= i+1; //再移动对应个像素点后继续
                        break;
                    }
                }
            }
            //不是噪点
            if (i > CHECK_DIS) {
                found_flag = 1;
                break;
            }
        }
    }
    //如果找到起始点则开始巡左线
    if (found_flag) {
        Lline_num = POINTS_MAX_LEN;
        //开始巡线
        findline_lefthand_adaptive(begin_x, begin_y);   //左手巡线(输入原始图像）
    }
    else Lline_num = 0;
}

// 寻找右起始点并巡线
void search_Rline(uint8_t height_start, uint8_t height_min){
    uint8_t begin_x,begin_y;
    bool found_flag;
    /*寻找右边边起始线*/
    begin_y = height_start - all_block_size/2 - 2;  //从指定起点开始向上
    begin_x = IMG_W - all_block_size/2 - 2;  //要比 IMG_W - all_block_size/2 - 1 小
    found_flag = 0;
    //如果该起始点为边线外则（即黑点）按x轴向内寻找
    if (IMG_AT(img_gray, begin_x, begin_y) <= start_thre){
        //最多到中心
        for(;begin_x>IMG_W/2+CAR_IMGAGE_W/2;begin_x--) {
            uint8_t i=0;
            //发现白点
            if (IMG_AT(img_gray, begin_x, begin_y) > start_thre){
                //向内再延伸check_dis个像素点 判断是否为噪点
                for (i=1;i<=CHECK_DIS;i++){
                    //为噪点
                    if (IMG_AT(img_gray, begin_x-i, begin_y) <= start_thre){
                        begin_x -= i+1; //再移动对应个像素点后继续
                        break;
                    }
                }
            }
            //不是噪点
            if (i > CHECK_DIS){
                found_flag = 1;
                break;
            }
        }
    }
    //否则（即白点）按y轴
    else{
        for (;begin_y > height_min;begin_y--){
            uint8_t i=0;
            //在阈值范围内 则判断是否为噪点
            if (IMG_AT(img_gray, begin_x, begin_y) <= start_thre){
                //向上再延伸check_dis个像素点 判断是否为噪点
                for (i=1;i<=CHECK_DIS;i++){
                    //为噪点
                    if (IMG_AT(img_gray, begin_x, begin_y-i) > start_thre){
                        begin_y -= i+1; //再移动对应个像素点后继续
                        break;
                    }
                }
            }
            //不是噪点
            if (i > CHECK_DIS) {
                found_flag = 1;
                break;
            }
        }
    }
    //如果找到则巡线
    if (found_flag) {
        Rline_num = POINTS_MAX_LEN;
        //开始巡线
        findline_righthand_adaptive(begin_x, begin_y);   //右手巡线(输入原始图像）
    }
    else Rline_num = 0;
}

// 对点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2]) {
    int half = all_block_size / 2;
    for (int i = 0; i < num; i++) {
        // 默认置点为(0,0)
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            //取点为 i-n/2...i-1 i i+2...i+n/2
            pts_out[i][0] += 1.0*pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += 1.0*pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        // 权重分布为 1 2 ...half+1... 2 1
        // 总权重为(1+2+...+half)*2+(half+1)=(half+1)half+(half+1)=(half+1)(half+1)
        pts_out[i][0] /= (half + 1) * (half + 1);
        pts_out[i][1] /= (half + 1) * (half + 1);
    }
}

// 点集等距采样,使走过的每段折线段的距离为固定距离
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2){
    float remain = 0.f; //两点间剩余要采样的长度
    float dist = sampled_dist*M2PIX; // 固定距离
    int len = 0;
    *num2 = POINTS_MAX_LEN;
    //由于用到i+1个in点,这里遍历到num1-1,且总点数要小于最大长度num2
    for(int i=0; i<num1-1 && len < *num2; i++){
        // 当前点坐标
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        // 下一个点和当前点坐标差值
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        // 两点距离,即初始要采样的距离
        float ds = sqrt(dx*dx+dy*dy);
        // 单位方向向量化
        dx /= ds; // dx=cosα
        dy /= ds; // dy=sinα

        // remain不足ds时进行插值
        while (remain < ds && len < *num2){
            // x,y按方向向量移动remain
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            ds -= remain;  // 总采样距离ds减少
            remain = dist; // remain固定为采样距离dist
        }
        remain -= ds;      // 为下次采样开头做准备
    }
    *num2 = len;
}

// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[]){
    // 首个点和最后一个点无法计算变化率,直接置为0
    angle_out[0] = 0;
    angle_out[num-1] = 0;
    // 对每个点,计算pin(i-angle_idx),pin(i+angle_idx)方向向量的夹角
    for (int i = 1; i < num-1; i++) {
         //计算pini和pin(i-dist)间的距离
        float dx0 = pts_in[i][0] - pts_in[clip(i - angle_idx, 0, num - 1)][0];
        float dy0 = pts_in[i][1] - pts_in[clip(i - angle_idx, 0, num - 1)][1];
        float ds0 = sqrtf(dx0 * dx0 + dy0 * dy0);

        //计算pin(i+dist)和pini间的距离
        float dx1 = pts_in[clip(i + angle_idx, 0, num - 1)][0] - pts_in[i][0];
        float dy1 = pts_in[clip(i + angle_idx, 0, num - 1)][1] - pts_in[i][1];
        float ds1 = sqrtf(dx1 * dx1 + dy1 * dy1);

        //计算两个方向向量的角度,此处也是将dx视作cos,dy视作sin
        dx0 /= ds0;
        dy0 /= ds0;
        dx1 /= ds1;
        dy1 /= ds1;

        //atan2f(y/x)严格计算夹角,这里实际上通过atan2f(sinθ/cosθ)=atan2f(tanθ)=θ来计算
        //其中二维向量可以写作三维:l0=(cos0,sin0,0),l1=(cos1,sin1,0),|l0|=|l1|=1
        //求叉积l0×l1=(0,0,cos0*sin1-cos1*sin0),取z分量就是sinθ(正负代表旋转方向)
        //求点积l0·l1=cos0*cos1+sin0*sin1,就是cosθ(正负代表左右象限)
        angle_out[i] = fabs(atan2f(dx0 * dy1 - dx1 * dy0, dx0 * dx1 + dy0 * dy1) / PI * 180);
    }
}

// 角度变化率非极大抑制,返回最大角及索引
void nms_angle(float angle_in[], int num, float *angle_max, int *idx) {
    *angle_max = 0;
    //末端角点无效
    num--;
    //从第2个点开始
    for (uint8_t i=1;i<num;i++){
        if (angle_in[i] > *angle_max) {
            *angle_max = angle_in[i];
            *idx = i;
        }
    }
}

// 从左边线跟踪中线
void track_leftline(float dist) {
    for (int i = 0; i < sampled_Lline_num; i++) {
        // 求解±approx_idx内
        float dx = sampled_Lline[clip(i + approx_idx, 0, sampled_Lline_num - 1)][0] 
                 - sampled_Lline[clip(i - approx_idx, 0, sampled_Lline_num - 1)][0];
        float dy = sampled_Lline[clip(i + approx_idx, 0, sampled_Lline_num - 1)][1]
                 - sampled_Lline[clip(i - approx_idx, 0, sampled_Lline_num - 1)][1];
        float ds = sqrt(dx * dx + dy * dy);
        dx /= ds;
        dy /= ds;
        // dist就是得到的像素距离下的半赛道长
        // 对(dx,dy)即(cosθ,sinθ)向左旋转90°
        // 得到(cos(θ+π/2),sin(θ+π/2))=(-sinθ,cosθ)=(-dy,dx)
        L2Mline[i][0] = sampled_Lline[i][0] - dy * dist;
        L2Mline[i][1] = sampled_Lline[i][1] + dx * dist;
    }
}

// 从右边线跟踪中线
void track_rightline(float dist) {
    for (int i = 0; i < sampled_Rline_num; i++) {
        float dx = sampled_Rline[clip(i + approx_idx, 0, sampled_Rline_num - 1)][0] 
                 - sampled_Rline[clip(i - approx_idx, 0, sampled_Rline_num - 1)][0];
        float dy = sampled_Rline[clip(i + approx_idx, 0, sampled_Rline_num - 1)][1] 
                 - sampled_Rline[clip(i - approx_idx, 0, sampled_Rline_num - 1)][1];
        float ds = sqrt(dx * dx + dy * dy);
        dx /= ds;
        dy /= ds;
        //(cos(θ-π/2),sin(θ-π/2))=(sinθ,-cosθ)=(dy,-dx)
        R2Mline[i][0] = sampled_Rline[i][0] + dy * dist;
        R2Mline[i][1] = sampled_Rline[i][1] - dx * dist;
    }
}

// 十字补线
void supplement_line(float pts_in[][2],int* num,int corner_index,float dist){
    // 水平向右为x轴,向量头作原点,从x轴逆时针旋转到该向量的角度为θ∈[0,π]
    // 统计0到corner_index-1构成向量的平均角度
    if (corner_index<=1) return; //去除无用补线

    avg_angle = 0;
    for (int i=0;i<corner_index-1;i++){
        float dx = pts_in[i+1][0]-pts_in[i][0];
        float dy = pts_in[i+1][1]-pts_in[i][1];
        avg_angle += -atan2f(dy,dx);
    }
    avg_angle/=(corner_index-1);

    // 无论如何,起始点都是角点,后续根据象限判断
    float start_x = pts_in[corner_index][0];
    float start_y = pts_in[corner_index][1];
    float abs_angle = fabs(avg_angle);

    // 下方线先垂直(π/2),即绝对平均角度扩展到45°~135°,向上补线
    if (abs_angle>PI/4 && abs_angle<3*PI/4){
        // 从角点开始,向上延伸至顶部且最多补num个点
        while (start_y>=0 && *num<60){
            start_x += dist*cos(avg_angle);
            start_y -= dist*sin(avg_angle);

            // 当补线点索引不超过数组长度时覆盖
            corner_index++;
            pts_in[corner_index][0]=start_x;
            pts_in[corner_index][1]=start_y;
            if (corner_index>*num) *num=*num+1;//超过时扩展数组
        }
        // 最后处理,如果向上补线后还是小于原来的num长度,将num截断
        // 向下补线是补到0为止,不做处理
        if (corner_index<*num) *num=corner_index;
    }
    // 上方线先水平(0或π),即绝对平均角度扩展到0~45°或135°~180°  
    else{
        //将预瞄点移至后面
        aim_point.idx = corner_index + 5;
    }
}

/*---------------------------角度计算-----------------------------*/
