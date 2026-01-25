#include "my_image_transmitter.hpp"



#define ZF_IMAGE_MAGIC 0x5A465644u // 'ZFVD'


// 构造 & 析构
zf_driver_image_client::zf_driver_image_client() : m_seq(0) {}
zf_driver_image_client::~zf_driver_image_client() {}

// 初始化
int8 zf_driver_image_client::init(const char *ip_addr, uint32 port)
{
    return m_tcp.init(ip_addr, port);
}

// 将 Mat 编码为 JPEG 并发送
int32 zf_driver_image_client::send_mat_as_jpeg(const cv::Mat &mat, int quality, int timeout_ms)
{
    if (mat.empty()) return -1;

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(quality);

    std::vector<uchar> buf;
    bool ok = cv::imencode(".jpg", mat, buf, params);
    if (!ok) return -1;

    zf_image_packet_header_t hdr;
    hdr.magic = htonl(ZF_IMAGE_MAGIC);
    hdr.format = htons(1); // JPEG
    hdr.reserved = 0;
    hdr.width = htonl((uint32_t)mat.cols);
    hdr.height = htonl((uint32_t)mat.rows);
    hdr.payload_len = htonl((uint32_t)buf.size());
    hdr.seq = htonl(++m_seq);

    // 先发送头部，再发送数据
    int32 sent = 0;
    int32 s = m_tcp.send_data_all(reinterpret_cast<const uint8*>(&hdr), sizeof(hdr), timeout_ms);
    if (s <= 0) return -1;
    sent += s;

    s = m_tcp.send_data_all(buf.data(), (uint32_t)buf.size(), timeout_ms);
    if (s < 0) return -1;
    sent += s;
    return sent;
}


// 在头文件中定义常量
#define IMG_CHANNEL_ID    1       // 图片通道ID，可根据需要修改
#define IMG_FORMAT_GRAY8  1       // 8位灰度图格式码
#define JUSTFLOAT_END     0x7F800000  // JustFloat前导帧结尾标识

/*********************************************************************************************************************
 * 函数简介     发送灰度原始数据（按照JustFloat协议）
 * 参数说明     data         灰度图像数据指针
 * 参数说明     width        图像宽度
 * 参数说明     height       图像高度
 * 参数说明     timeout_ms   超时时间(毫秒)
 * 返回参数     int32        成功发送的总字节数，-1表示失败
 * 备注信息     使用JustFloat协议格式，先发送前导帧，再发送图像数据
 ********************************************************************************************************************/
int32 zf_driver_image_client::send_gray_raw(const uint8_t *data, uint32 width, uint32 height, int timeout_ms)
{
    if (!data || width == 0 || height == 0) {
        printf("错误：无效的图像参数\r\n");
        return -1;
    }

    // 计算图像数据大小（检查溢出）
    uint64_t payload_len64 = (uint64_t)width * (uint64_t)height;
    if (payload_len64 == 0 || payload_len64 > 0xFFFFFFFFu) {
        printf("错误：图像尺寸过大或为0\r\n");
        return -1;
    }
    uint32_t image_size = (uint32_t)payload_len64;

    // 构造前导帧（7个32位单元）
    // 协议要求：int preFrame[7] = { IMG_ID, IMG_SIZE, IMG_WIDTH, IMG_HEIGHT, IMG_FORMAT, 0x7F800000, 0x7F800000 };
    uint32_t preFrame[7];
    preFrame[0] = (uint32_t)IMG_CHANNEL_ID;  // IMG_ID
    preFrame[1] = (uint32_t)image_size;       // IMG_SIZE (bytes)
    preFrame[2] = (uint32_t)width;            // IMG_WIDTH
    preFrame[3] = (uint32_t)height;           // IMG_HEIGHT
    preFrame[4] = (uint32_t)IMG_FORMAT_GRAY8; // IMG_FORMAT
    preFrame[5] = JUSTFLOAT_END;                // JustFloat end marker (+Inf bit pattern)
    preFrame[6] = JUSTFLOAT_END;                // JustFloat end marker (+Inf bit pattern)

    // 为了兼容不同主机字节序，显式把前导帧转换为 little-endian（FireWater 常以 little-endian 位模式解析）
    auto to_le32 = [](uint32_t v)->uint32_t {
    #if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
        return __builtin_bswap32(v);
    #else
        return v;
    #endif
    };

    uint32_t preFrame_le[7];
    for (int i = 0; i < 7; ++i) preFrame_le[i] = to_le32(preFrame[i]);

    // 发送前导帧（28 字节）
    int32_t header_sent = m_tcp.send_data_all(reinterpret_cast<const uint8_t*>(preFrame_le),
                                              sizeof(preFrame_le),
                                              timeout_ms);
    if (header_sent != (int32_t)sizeof(preFrame_le)) {
        printf("发送前导帧失败: 期望 %zu 字节，实际 %d 字节\r\n",
               sizeof(preFrame_le), header_sent);
        return -1;
    }

    // 发送图像数据（灰度 8bit 连续数据）
    int32_t data_sent = m_tcp.send_data_all(data, image_size, timeout_ms);
    if (data_sent != (int32_t)image_size) {
        printf("发送图像数据失败: 期望 %u 字节，实际 %d 字节\r\n",
               image_size, data_sent);
        return -1;
    }

    int32_t total_sent = header_sent + data_sent;
    m_seq++;

    // 可选调试输出
    if ((m_seq % 10) == 0) {
        printf("JustFloat 发送: ch=%d size=%u w=%u h=%u fmt=%d total=%d\r\n",
               IMG_CHANNEL_ID, image_size, width, height, IMG_FORMAT_GRAY8, total_sent);
    }

    return total_sent;
}