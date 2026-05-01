/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/
#include "my_menu.hpp"
#include "my_task_function.hpp"
#include "zf_driver_delay.hpp"

#include <cstring>

// 全局菜单实例指针
MyMenu* g_menu_instance = nullptr;

/**
 * @brief 菜单 UI 移植辅助区域开始
 * @note 从此标志到“菜单 UI 移植辅助区域结束”的代码只依赖 zf_device_ips200 的基础绘图接口。
 */
// ==================== MENU_UI_PORTABLE_HELPER_BEGIN ====================

/**
 * @brief 菜单绘制辅助函数集中区
 * @note 这些函数只服务于上层菜单 UI，不写入 zf_device_ips200 底层库。
 *       若移植到不支持 framebuffer 或 fill_rect 的屏幕库，可优先替换或删除本区域。
 */
namespace {
constexpr uint16 MENU_TTF_ITEM_HEIGHT = ITEM_H;
constexpr uint16 MENU_STATIC_ITEM_HEIGHT = 24;
constexpr uint16 MENU_ITEM_MARGIN_X = 4;
constexpr uint16 MENU_TEXT_X = 12;
constexpr uint16 MENU_SELECT_RADIUS = 6;
constexpr uint8 MENU_ANIMATION_FRAMES = 9;
constexpr uint16 MENU_ANIMATION_DELAY_MS = 22;
constexpr uint16 MENU_SELECT_PADDING_X = 6;
constexpr uint16 MENU_SELECT_PADDING_Y = 2;

struct menu_level_info_t {
    int page_start;
    int selected_index;
    int selected_row;
};

struct menu_render_style_t {
    bool use_ttf;
    uint16 item_height;
    uint16 text_y_offset;
    uint16 max_text_chars;
    uint16 text_width;
    uint16 text_height;
};

struct menu_highlight_rect_t {
    int x;
    int y;
    int w;
    int h;
};

/**
 * @brief 统计当前菜单层级信息
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param selected_menu 当前选中项
 * @return 当前选中项在本层级中的序号、分页起点和显示行号
 */
menu_level_info_t get_menu_level_info(const Menu *menu_table, int table_size, const Menu *selected_menu)
{
    menu_level_info_t info = {0, -1, 0};
    int current_index = 0;
    const int parent_id = selected_menu->parent_id;

    for (int i = 0; i < table_size; i++)
    {
        if (menu_table[i].parent_id == parent_id)
        {
            if (&menu_table[i] == selected_menu)
                info.selected_index = current_index;
            current_index++;
        }
    }

    if (info.selected_index >= MENU_MAX_ROW)
        info.page_start = info.selected_index - (MENU_MAX_ROW - 1);

    info.selected_row = info.selected_index - info.page_start;
    if (info.selected_row < 0)
        info.selected_row = 0;

    return info;
}

/**
 * @brief 以当前显示库已有的 fill_rect 绘制圆角矩形
 * @param ips_display 屏幕对象
 * @param x 矩形左上角 X 坐标
 * @param y 矩形左上角 Y 坐标
 * @param w 矩形宽度
 * @param h 矩形高度
 * @param radius 圆角半径
 * @param color RGB565 填充颜色
 * @note 该函数通过逐行短矩形近似圆角，不要求底层库提供专用圆角 API。
 */
void draw_menu_round_rect(zf_device_ips200 *ips_display, uint16 x, uint16 y,
                          uint16 w, uint16 h, uint16 radius, uint16 color)
{
    if (!ips_display || w == 0 || h == 0)
        return;

    if (radius * 2 > h)
        radius = h / 2;
    if (radius * 2 > w)
        radius = w / 2;

    for (uint16 row = 0; row < h; row++)
    {
        uint16 inset = 0;

        if (row < radius)
        {
            const int dy = radius - 1 - row;
            inset = static_cast<uint16>((dy * dy) / (radius ? radius : 1));
        }
        else if (row >= h - radius)
        {
            const int dy = row - (h - radius);
            inset = static_cast<uint16>((dy * dy) / (radius ? radius : 1));
        }

        if (inset * 2 >= w)
            continue;

        ips_display->fill_rect(x + inset, y + row, w - inset * 2, 1, color);
    }
}

/**
 * @brief 将菜单名截断到静态缓冲区，防止文本超出屏幕
 * @param dst 输出缓冲区
 * @param dst_size 输出缓冲区大小
 * @param src 原始菜单名
 * @param max_chars 最大显示字符数
 * @note 菜单名当前为 ASCII，静态字库与 TTF 分支共用该截断逻辑，保持布局一致。
 */
void copy_menu_label(char *dst, size_t dst_size, const char *src, uint16 max_chars)
{
    if (!dst || dst_size == 0)
        return;

    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';

    if (max_chars == 0 || strlen(dst) <= max_chars
        || static_cast<size_t>(max_chars) + 1 > dst_size)
        return;

    if (max_chars > 3)
    {
        dst[max_chars - 3] = '.';
        dst[max_chars - 2] = '.';
        dst[max_chars - 1] = '.';
        dst[max_chars] = '\0';
    }
    else
    {
        dst[max_chars] = '\0';
    }
}

/**
 * @brief 计算菜单名对应的选中色块矩形
 * @param ips_display 屏幕对象
 * @param style 菜单绘制风格
 * @param display_row 当前页显示行号
 * @param label 已截断后的菜单名
 * @return 仅包裹菜单名文字和少量内边距的色块矩形
 */
menu_highlight_rect_t get_menu_highlight_rect(zf_device_ips200 *ips_display,
                                              const menu_render_style_t &style,
                                              int display_row, const char *label)
{
    const size_t label_len = label ? strlen(label) : 0;
    const uint16 max_w = static_cast<uint16>(ips_display->get_width() - MENU_ITEM_MARGIN_X * 2);
    uint16 text_w = static_cast<uint16>(label_len * style.text_width);
    uint16 rect_w = static_cast<uint16>(text_w + MENU_SELECT_PADDING_X * 2);
    uint16 rect_h = static_cast<uint16>(style.text_height + MENU_SELECT_PADDING_Y * 2);

    if (rect_w > max_w)
        rect_w = max_w;
    if (rect_h > style.item_height - 2)
        rect_h = style.item_height - 2;

    menu_highlight_rect_t rect = {};
    rect.x = MENU_TEXT_X - MENU_SELECT_PADDING_X;
    rect.y = display_row * style.item_height + style.text_y_offset - MENU_SELECT_PADDING_Y;
    rect.w = rect_w;
    rect.h = rect_h;

    if (rect.x < MENU_ITEM_MARGIN_X)
        rect.x = MENU_ITEM_MARGIN_X;
    if (rect.x + rect.w > ips_display->get_width() - MENU_ITEM_MARGIN_X)
        rect.x = ips_display->get_width() - MENU_ITEM_MARGIN_X - rect.w;
    if (rect.y < display_row * style.item_height)
        rect.y = display_row * style.item_height;
    if (rect.y + rect.h > (display_row + 1) * style.item_height)
        rect.y = (display_row + 1) * style.item_height - rect.h;

    return rect;
}

void clamp_menu_highlight_rect(zf_device_ips200 *ips_display, menu_highlight_rect_t &rect)
{
    if (rect.w < 1)
        rect.w = 1;
    if (rect.h < 1)
        rect.h = 1;
    if (rect.w > ips_display->get_width())
        rect.w = ips_display->get_width();
    if (rect.h > ips_display->get_height())
        rect.h = ips_display->get_height();

    if (rect.x < 0)
        rect.x = 0;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.x + rect.w > ips_display->get_width())
        rect.x = ips_display->get_width() - rect.w;
    if (rect.y + rect.h > ips_display->get_height())
        rect.y = ips_display->get_height() - rect.h;
}

void clear_menu_highlight_rect(zf_device_ips200 *ips_display, const menu_highlight_rect_t &rect)
{
    int x = rect.x - 1;
    int y = rect.y - 1;
    int w = rect.w + 2;
    int h = rect.h + 2;

    if (x < 0)
    {
        w += x;
        x = 0;
    }
    if (y < 0)
    {
        h += y;
        y = 0;
    }
    if (x + w > ips_display->get_width())
        w = ips_display->get_width() - x;
    if (y + h > ips_display->get_height())
        h = ips_display->get_height() - y;
    if (w <= 0 || h <= 0)
        return;

    ips_display->fill_rect(static_cast<uint16>(x), static_cast<uint16>(y),
                           static_cast<uint16>(w), static_cast<uint16>(h),
                           MENU_BG_COLOR);
}

/**
 * @brief 带轻微回弹的插值因子，单位为千分比
 * @param frame 当前帧序号，从 1 开始
 * @return 插值进度，1000 为目标值，允许中间帧略微越过目标形成弹性
 */
int get_elastic_progress_permille(uint8 frame)
{
    static const int progress[] = {180, 390, 620, 840, 1030, 1110, 1060, 1015, 1000};
    if (frame == 0)
        return 0;
    if (frame > MENU_ANIMATION_FRAMES)
        return 1000;
    return progress[frame - 1];
}

int elastic_lerp(int start, int end, int progress_permille)
{
    return start + (end - start) * progress_permille / 1000;
}

/**
 * @brief 绘制单个菜单项文字
 * @param ips_display 屏幕对象
 * @param style 菜单绘制风格
 * @param x 文本 X 坐标
 * @param y 文本 Y 坐标
 * @param color 文本颜色
 * @param bg_color 静态字库使用的背景色
 * @param text 文本内容
 */
void draw_menu_text(zf_device_ips200 *ips_display, const menu_render_style_t &style,
                    uint16 x, uint16 y, uint16 color, uint16 bg_color, const char *text)
{
    if (style.use_ttf)
    {
        ips_display->print(x, y, color, MENU_FONT_SIZE, text);
        return;
    }

    const uint16 old_pen_color = ips_display->pen_color;
    const uint16 old_bg_color = ips_display->bg_color;
    ips_display->pen_color = color;
    ips_display->bg_color = bg_color;
    ips_display->show_string(x, y, text);
    ips_display->pen_color = old_pen_color;
    ips_display->bg_color = old_bg_color;
}

/**
 * @brief 绘制菜单的一帧画面
 * @param ips_display 屏幕对象
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param selected_menu 当前选中项
 * @param info 当前层级信息
 * @param style 绘制风格
 * @param select_y 选中色块的 Y 坐标，用于动画插值
 * @param highlight_text 是否用选中态颜色重绘选中项文字
 */
void render_menu_frame(zf_device_ips200 *ips_display, const Menu *menu_table, int table_size,
                       const Menu *selected_menu, const menu_level_info_t &info,
                       const menu_render_style_t &style, const menu_highlight_rect_t &highlight_rect,
                       bool highlight_text)
{
    ips_display->full(MENU_BG_COLOR);

    draw_menu_round_rect(ips_display, static_cast<uint16>(highlight_rect.x),
                         static_cast<uint16>(highlight_rect.y),
                         static_cast<uint16>(highlight_rect.w),
                         static_cast<uint16>(highlight_rect.h),
                         MENU_SELECT_RADIUS, MENU_SELECT_COLOR);

    int current_item_count = 0;
    int display_row = 0;

    for (int i = 0; i < table_size; i++)
    {
        if (menu_table[i].parent_id == selected_menu->parent_id)
        {
            if (current_item_count >= info.page_start && display_row < MENU_MAX_ROW)
            {
                const uint16 y_offset = display_row * style.item_height;
                const bool is_selected = (&menu_table[i] == selected_menu);
                char menu_name_buf[32];

                copy_menu_label(menu_name_buf, sizeof(menu_name_buf),
                                menu_table[i].name, style.max_text_chars);

                if (!is_selected || !highlight_text)
                {
                    draw_menu_text(ips_display, style, MENU_TEXT_X, y_offset + style.text_y_offset,
                                   MENU_TEXT_COLOR, MENU_BG_COLOR, menu_name_buf);
                }
                else
                {
                    draw_menu_text(ips_display, style, MENU_TEXT_X, y_offset + style.text_y_offset,
                                   MENU_SELECT_TEXT, MENU_SELECT_COLOR, menu_name_buf);
                }

                if (!is_selected)
                    ips_display->fill_rect(MENU_ITEM_MARGIN_X, y_offset + style.item_height - 1,
                                           ips_display->get_width() - MENU_ITEM_MARGIN_X * 2,
                                           1, MENU_DIVIDER_COLOR);
                display_row++;
            }
            current_item_count++;
        }
    }

    ips_display->update();
}

/**
 * @brief 查找当前页指定显示行对应的菜单项
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param parent_id 当前层级父 ID
 * @param page_start 当前页起始序号
 * @param display_row 当前页显示行号
 * @return 对应菜单项；超出范围时返回 nullptr
 */
const Menu *find_menu_by_display_row(const Menu *menu_table, int table_size,
                                     int parent_id, int page_start, int display_row)
{
    int current_item_count = 0;

    for (int i = 0; i < table_size; i++)
    {
        if (menu_table[i].parent_id == parent_id)
        {
            if (current_item_count == page_start + display_row)
                return &menu_table[i];
            current_item_count++;
        }
    }

    return nullptr;
}

/**
 * @brief 重绘指定行范围内的菜单项普通状态
 * @param ips_display 屏幕对象
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param selected_menu 当前选中项
 * @param info 当前层级信息
 * @param style 绘制风格
 * @param first_row 起始显示行
 * @param last_row 结束显示行
 * @note 动画过程中只重绘受选中色块影响的局部行，避免每个插帧都重绘全屏。
 */
void repaint_menu_rows_normal(zf_device_ips200 *ips_display, const Menu *menu_table, int table_size,
                              const Menu *selected_menu, const menu_level_info_t &info,
                              const menu_render_style_t &style, int first_row, int last_row)
{
    if (first_row < 0)
        first_row = 0;

    const int screen_last_row = (ips_display->get_height() - 1) / style.item_height;
    if (last_row > screen_last_row)
        last_row = screen_last_row;

    for (int row = first_row; row <= last_row; row++)
    {
        const uint16 y_offset = static_cast<uint16>(row * style.item_height);
        uint16 repaint_h = style.item_height;
        if (y_offset + repaint_h > ips_display->get_height())
            repaint_h = static_cast<uint16>(ips_display->get_height() - y_offset);

        ips_display->fill_rect(0, y_offset, ips_display->get_width(), repaint_h, MENU_BG_COLOR);

        const Menu *row_menu = find_menu_by_display_row(menu_table, table_size,
                                                        selected_menu->parent_id,
                                                        info.page_start, row);
        if (!row_menu)
            continue;

        char menu_name_buf[32];

        copy_menu_label(menu_name_buf, sizeof(menu_name_buf),
                        row_menu->name, style.max_text_chars);

        if (row_menu != selected_menu)
            ips_display->fill_rect(MENU_ITEM_MARGIN_X, y_offset + style.item_height - 1,
                                   ips_display->get_width() - MENU_ITEM_MARGIN_X * 2,
                                   1, MENU_DIVIDER_COLOR);
        draw_menu_text(ips_display, style, MENU_TEXT_X, y_offset + style.text_y_offset,
                       MENU_TEXT_COLOR, MENU_BG_COLOR, menu_name_buf);
    }
}

/**
 * @brief 绘制一个局部插帧
 * @param ips_display 屏幕对象
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param selected_menu 当前选中项
 * @param info 当前层级信息
 * @param style 绘制风格
 * @param previous_rect 上一帧选中色块矩形
 * @param select_rect 当前帧选中色块矩形
 * @param highlight_text 是否绘制最终选中文字状态
 * @note 该函数只清理选中色块上一帧和当前帧覆盖的行，降低 TTF 模式下的插帧开销。
 */
void render_menu_animation_frame(zf_device_ips200 *ips_display, const Menu *menu_table, int table_size,
                                 const Menu *selected_menu, const menu_level_info_t &info,
                                 const menu_render_style_t &style,
                                 const menu_highlight_rect_t &previous_rect,
                                 const menu_highlight_rect_t &select_rect, bool highlight_text)
{
    const int min_y = previous_rect.y < select_rect.y ? previous_rect.y : select_rect.y;
    const int previous_bottom = previous_rect.y + previous_rect.h;
    const int current_bottom = select_rect.y + select_rect.h;
    const int max_y = previous_bottom > current_bottom ? previous_bottom : current_bottom;
    const int first_row = min_y / style.item_height;
    const int last_row = (max_y + 1) / style.item_height;

    clear_menu_highlight_rect(ips_display, previous_rect);
    clear_menu_highlight_rect(ips_display, select_rect);

    repaint_menu_rows_normal(ips_display, menu_table, table_size, selected_menu,
                             info, style, first_row, last_row);

    draw_menu_round_rect(ips_display, static_cast<uint16>(select_rect.x),
                         static_cast<uint16>(select_rect.y),
                         static_cast<uint16>(select_rect.w),
                         static_cast<uint16>(select_rect.h),
                         MENU_SELECT_RADIUS, MENU_SELECT_COLOR);

    if (highlight_text)
    {
        char menu_name_buf[32];
        const uint16 selected_text_y = static_cast<uint16>(info.selected_row * style.item_height
                                                           + style.text_y_offset);

        copy_menu_label(menu_name_buf, sizeof(menu_name_buf),
                        selected_menu->name, style.max_text_chars);
        draw_menu_text(ips_display, style, MENU_TEXT_X, selected_text_y,
                       MENU_SELECT_TEXT, MENU_SELECT_COLOR, menu_name_buf);
    }

    ips_display->update();
}

/**
 * @brief 生成菜单风格参数
 * @param ips_display 屏幕对象
 * @param use_ttf 是否使用 TTF 字库
 * @return 当前字库模式对应的布局参数
 */
menu_render_style_t get_menu_render_style(zf_device_ips200 *ips_display, bool use_ttf)
{
    menu_render_style_t style = {};
    style.use_ttf = use_ttf;
    style.item_height = use_ttf ? MENU_TTF_ITEM_HEIGHT : MENU_STATIC_ITEM_HEIGHT;
    style.text_y_offset = use_ttf ? 3 : (MENU_STATIC_ITEM_HEIGHT - MENU_STATIC_FONT_HEIGHT) / 2;
    style.text_width = use_ttf ? 12 : MENU_STATIC_FONT_WIDTH;
    style.text_height = use_ttf ? static_cast<uint16>(MENU_FONT_SIZE) : MENU_STATIC_FONT_HEIGHT;

    const uint16 text_right_margin = MENU_ITEM_MARGIN_X;
    if (use_ttf)
        style.max_text_chars = (ips_display->get_width() - MENU_TEXT_X - text_right_margin) / 11;
    else
        style.max_text_chars = (ips_display->get_width() - MENU_TEXT_X - text_right_margin)
                               / MENU_STATIC_FONT_WIDTH;

    return style;
}

/**
 * @brief 绘制带选中色块移动动画的菜单页面
 * @param ips_display 屏幕对象
 * @param menu_table 菜单表
 * @param table_size 菜单表长度
 * @param selected_menu 当前选中项
 * @param use_ttf 是否使用 TTF 字库
 */
void draw_menu_with_animation(zf_device_ips200 *ips_display, const Menu *menu_table,
                              int table_size, const Menu *selected_menu, bool use_ttf)
{
    static bool has_last_menu_state = false;
    static int last_parent_id = -10000;
    static int last_page_start = 0;
    static int last_selected_row = 0;
    static menu_highlight_rect_t last_highlight_rect = {};

    const menu_level_info_t info = get_menu_level_info(menu_table, table_size, selected_menu);
    const menu_render_style_t style = get_menu_render_style(ips_display, use_ttf);
    char selected_label[32];
    copy_menu_label(selected_label, sizeof(selected_label), selected_menu->name, style.max_text_chars);
    const menu_highlight_rect_t target_rect = get_menu_highlight_rect(ips_display, style,
                                                                      info.selected_row,
                                                                      selected_label);
    const bool can_animate = has_last_menu_state
                             && last_parent_id == selected_menu->parent_id
                             && last_page_start == info.page_start
                             && last_selected_row != info.selected_row;

    if (can_animate)
    {
        menu_highlight_rect_t previous_rect = last_highlight_rect;

        for (uint8 frame = 1; frame <= MENU_ANIMATION_FRAMES; frame++)
        {
            const int progress = get_elastic_progress_permille(frame);
            menu_highlight_rect_t select_rect = {};
            select_rect.x = elastic_lerp(last_highlight_rect.x, target_rect.x, progress);
            select_rect.y = elastic_lerp(last_highlight_rect.y, target_rect.y, progress);
            select_rect.w = elastic_lerp(last_highlight_rect.w, target_rect.w, progress);
            select_rect.h = elastic_lerp(last_highlight_rect.h, target_rect.h, progress);
            clamp_menu_highlight_rect(ips_display, select_rect);
            render_menu_animation_frame(ips_display, menu_table, table_size, selected_menu,
                                        info, style, previous_rect, select_rect,
                                        frame == MENU_ANIMATION_FRAMES);
            previous_rect = select_rect;
            system_delay_ms(MENU_ANIMATION_DELAY_MS);
        }
    }
    else
    {
        render_menu_frame(ips_display, menu_table, table_size, selected_menu,
                          info, style, target_rect, true);
    }

    has_last_menu_state = true;
    last_parent_id = selected_menu->parent_id;
    last_page_start = info.page_start;
    last_selected_row = info.selected_row;
    last_highlight_rect = target_rect;
}
}

// ===================== MENU_UI_PORTABLE_HELPER_END =====================

Menu MyMenu::menu_table[] = {
    // ID, ParentID, Name, Function
    { 0 ,-1,"Main Menu",nullptr },
    { 1 ,0,"mode1",nullptr},
    { 2 ,0,"mode2",nullptr},
    { 3 ,0,"mode3",nullptr},
    { 4 ,0,"mode4",nullptr},
    { 5 ,0,"mode5",nullptr},
    { 6 ,0,"mode6",nullptr},
    { 7 ,0,"mode7",nullptr},
    { 8 ,1,"key_remap_test",MyMenu::key_remap_test},
    { 9 ,1,"imu_angle_display",MyMenu::imu_angle_display},
    { 10,2,"brushless_calibration",MyMenu::brushless_calibration},
    { 11,0,"anan is cute",nullptr}
};

//计算table大小
const int MyMenu::MENU_TABLE_SIZE = sizeof(MyMenu::menu_table) / sizeof(Menu);

MyMenu::MyMenu(MyKey* key_mgr, zf_device_ips200* ips_disp) 
    : key_manager(key_mgr), ips_display(ips_disp), mode_inter_flag(0) 
{
    //赋值操作，硬件初始化在init函数中进行
    g_menu_instance = this;
    current_menu = &menu_table[0]; 
}

void MyMenu::init(){
    g_menu_instance->ips_display->init(IPS_DEVICE_PATH);
    g_menu_instance->ips_display->clear();
    draw_menu(current_menu);
    g_menu_instance->ips_display->update();

}

// 根据 ID 查找菜单项指针
Menu* MyMenu::find_menu_by_id(int id) {
    if (id == -1) return nullptr;
    for (int i = 0; i < MENU_TABLE_SIZE; i++) {
        if (menu_table[i].id == id) return &menu_table[i];
    }
    return nullptr;
}

Menu* MyMenu::get_first_child(int parent_id) {
    for (int i = 0; i < MENU_TABLE_SIZE; i++) {
        if (menu_table[i].parent_id == parent_id) return &menu_table[i];
    }
    return nullptr;
}

Menu* MyMenu::menu_navigate(Menu *current, MenuAction action)
{
    if (!current) return &menu_table[0];
    int curr_idx = current - menu_table; // 计算当前指针在数组中的下标

    switch (action)
    {
    case MENU_UP:
    {
        // 向上找：在当前项之前，寻找第一个 parent_id 相同的项
        for (int i = curr_idx - 1; i >= 0; i--) {
            if (menu_table[i].parent_id == current->parent_id) {
                return &menu_table[i];
            }
        }
        break; // 没找到，维持现状
    }
    case MENU_DOWN:
    {
        // 向下找：在当前项之后，寻找第一个 parent_id 相同的项
        for (int i = curr_idx + 1; i < MENU_TABLE_SIZE; i++) {
            if (menu_table[i].parent_id == current->parent_id) {
                return &menu_table[i];
            }
        }
        break;
    }
    case MENU_OK:
    {
        // 1. 优先检查是否有子菜单
        Menu* child = get_first_child(current->id);
        if (child) {
            return child; // 发现子菜单，进入下一层，不改变 flag
        }
        
        // 2. 如果没有子菜单，说明这是一个功能叶子节点
        if (current->handler != nullptr) {
            //修改标志位，进入菜单交互
            this->mode_inter_flag = 1; 
            ips_display->clear();

        }
        return current;
    }
    case MENU_BACK:
    {
        // 返回父节点
        Menu* parent = find_menu_by_id(current->parent_id);
        return parent ? parent : current;
    }
    default:
        break;
    }
    return current;
}


void MyMenu::draw_menu(Menu *selected_menu)
{
#if WHETHER_USE_TTF
    draw_menu_ttf(selected_menu);
#else
    draw_menu_static(selected_menu);
#endif
}

/**
 * @brief 使用动态 TTF 字库绘制当前层级菜单
 * @param selected_menu 当前选中的菜单项
 * @note TTF 模式可显示 UTF-8 字符，但菜单行高仍需大于字号，避免相邻字形上下重叠。
 */
void MyMenu::draw_menu_ttf(Menu *selected_menu)
{
    if (!selected_menu) return;
    draw_menu_with_animation(ips_display, menu_table, MENU_TABLE_SIZE, selected_menu, true);
}

/**
 * @brief 使用静态 8x16 ASCII 字库绘制当前层级菜单
 * @param selected_menu 当前选中的菜单项
 * @note 静态字库不支持 UTF-8 汉字，菜单名称应使用 ASCII；本函数通过固定行高和字符宽度避免文字重叠。
 */
void MyMenu::draw_menu_static(Menu *selected_menu)
{
    if (!selected_menu) return;
    draw_menu_with_animation(ips_display, menu_table, MENU_TABLE_SIZE, selected_menu, false);
}

void MyMenu::menu_system(void)
{
    uint8 cl_action = key_manager->get_menu_action(); // 获取菜单动作
    if (cl_action == WAITING) return;                 // 无按键则跳过

    if (mode_inter_flag == 0)
    {
        uint8 pre_flag = mode_inter_flag; 
        
        // 1. 正常的菜单导航模式
        Menu* next_menu = menu_navigate(current_menu, (MenuAction)cl_action);
        
        // 如果菜单项发生了切换，刷新菜单界面
        if (next_menu != current_menu) {
            current_menu = next_menu;
            draw_menu(current_menu);
        }
        else if (mode_inter_flag != pre_flag && mode_inter_flag == 1) {
            if (current_menu->handler != nullptr) {
                // 此时 flag 刚变 1，通常需要立即执行一次 handler(ENTER) 来初始化功能界面
                current_menu->handler(cl_action);
            }
        }
    }
    else
    {   
        uint8 pre_flag = mode_inter_flag;
        
        // 2. 模式交互模式：直接调用当前菜单绑定的函数
        if (current_menu->handler != nullptr) {
            current_menu->handler(cl_action);
        } else {
            mode_inter_flag = 0; // 防御性逻辑
        }

        // 退出交互模式回到菜单导航时，立即刷新菜单页面
        if (pre_flag == 1 && mode_inter_flag == 0) {
            draw_menu(current_menu); 
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前菜单
// 参数说明 无
// 返回参数 Menu* 当前菜单指针
// 使用示例 Menu* current = menu_system.get_current_menu();
// 备注信息 获取当前选中的菜单项
//-------------------------------------------------------------------------------------------------------------------
Menu* MyMenu::get_current_menu(void)
{
    return current_menu;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置模式交互标志
// 参数说明 flag 标志值
// 返回参数 无
// 使用示例 menu_system.set_mode_inter_flag(1);
// 备注信息 设置是否进入模式交互状态
//-------------------------------------------------------------------------------------------------------------------
void MyMenu::set_mode_inter_flag(uint8 flag)
{
    mode_inter_flag = flag;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取模式交互标志
// 参数说明 无
// 返回参数 uint8 当前标志值
// 使用示例 uint8 flag = menu_system.get_mode_inter_flag();
// 备注信息 获取当前模式交互状态
//-------------------------------------------------------------------------------------------------------------------
uint8 MyMenu::get_mode_inter_flag(void)
{
    return mode_inter_flag;
}

// 菜单功能函数，依照函数功能，注意维护id-------------------------------------------------------------------
void MyMenu::menu_mode_0(uint8 cl_action)
{
    if(g_menu_instance==nullptr) return;
    // 模式0的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式0
        break;
    case 1: // 上键
        break;
    case 2: // 下键
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }  

    tracking();
}

void MyMenu::menu_mode_1(uint8 cl_action)
{
    if(g_menu_instance==nullptr) return;
    // 模式1的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式1
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    // 显示模式1信息
    // send_picture_to_Serve();
    // tracking();
}

void MyMenu::menu_mode_2(uint8 cl_action)
{
    // 模式2的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式2
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
}

void MyMenu::menu_mode_3(uint8 cl_action)
{
    // 模式3的具体实现
    switch (cl_action)
    {
    case 0: // 确认键
        // 启动模式3
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式3信息
    g_menu_instance->ips_display->clear();
    g_menu_instance->ips_display->show_string(0, 0, "Mode 3 Active");
}

void MyMenu::key_remap_test(uint8 cl_action)
{
    static int temp_test = 0;
    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: 
        temp_test++;
        break;
    case 1:
        temp_test--;
        break;
    case 2:
        temp_test = 0;
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
    
    // 显示模式4信息
    g_menu_instance->ips_display->clear();
    g_menu_instance->ips_display->print(0, 0, "this is model 4!");
    g_menu_instance->ips_display->print(0, 16, "key remap test:1.+;2.-;");
    g_menu_instance->ips_display->print(0, 32, "     3.set 0;4.return;");
    g_menu_instance->ips_display->print(0,3*16,"-------\ntest num:%d\n--------",temp_test);
    g_menu_instance->ips_display->print(0,7*16,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24,DEFAULT_PENCOLOR,36,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24+36,DEFAULT_PENCOLOR,48,"你好，世界");
    g_menu_instance->ips_display->print(0,7*16+24+36+48,DEFAULT_PENCOLOR,64,"你好，世界");
    g_menu_instance->ips_display->update();
    
}

void MyMenu::menu_mode_6(uint8 cl_action)
{

    // 模式4的具体实现
    switch (cl_action)
    {
    case 0: // 切换键
        break;
    case 1: // +
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
}


void MyMenu::imu_angle_display(uint8 cl_action)
{
    // IMU角度显示模式
    switch (cl_action)
    {
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;
    default:
        break;
    }
}

// 惯性导航地图记录界面
void MyMenu::get_map(uint8 cl_action) {
    if (!path_tracker_component.is_recording) {
        g_menu_instance->ips_display->show_string(1,1,"1, record");
        g_menu_instance->ips_display->show_string(1,1+1*16,"2, clear_map");
        g_menu_instance->ips_display->show_string(1,1+2*16,"3, insert_map");
        g_menu_instance->ips_display->show_string(1,1+4*16,"4, return");
        g_menu_instance->ips_display->update();
    }

    switch (cl_action) {
        case 0: // 记录开关
            if (!path_tracker_component.is_recording) {
                ahrs.reset(); // 重置姿态，确保航向角从0开始
                system_delay_ms(500);
                path_tracker_component.start_remember();
            } else {
                path_tracker_component.stop_remember(false);
            }
            break;

        case 1: // 清空内存中的地图数据
            path_tracker_component.stop_remember(true);
            break;

        case 2: // 转换、平滑并保存地图
            if(!path_tracker_component.is_recording){
                if (path_tracker_component.current_index > 5) {
                    // 1. 先进行高斯滤波，消除编码器和传感器的原始噪点
                    path_tracker_component.apply_gaussian_filter(1.2f, 5);

                    // 2. 将 PathTracker 中的原始点提取并转换为“里程-坐标”序列
                    std::vector<double> s_list; // 累计里程（自变量）
                    std::vector<double> x_list; // X 坐标
                    std::vector<double> y_list; // Y 坐标
                    double total_s = 0;

                    // 插入起点
                    s_list.push_back(0);
                    x_list.push_back(path_tracker_component.path_array[0].x);
                    y_list.push_back(path_tracker_component.path_array[0].y);

                    // 遍历记录的所有点，计算累计里程 s
                    for (int i = 1; i < path_tracker_component.current_index; i++) {
                        double dx = path_tracker_component.path_array[i].x - path_tracker_component.path_array[i-1].x;
                        double dy = path_tracker_component.path_array[i].y - path_tracker_component.path_array[i-1].y;
                        total_s += std::sqrt(dx * dx + dy * dy);

                        s_list.push_back(total_s);
                        x_list.push_back(path_tracker_component.path_array[i].x);
                        y_list.push_back(path_tracker_component.path_array[i].y);
                    }

                    // 3. 使用 Akima 进行插值计算
                    AkimaInterpolator x_engine, y_engine;
                    if (x_engine.compute(s_list, x_list) && y_engine.compute(s_list, y_list)) {
                        // 4. 等距采样并保存为文件
                        AkimaInterpolator::save_as_tracking_map(
                            x_engine, 
                            y_engine, 
                            total_s, 
                            10, 
                            "tracking_map.txt"
                        );
                        AkimaInterpolator::convert_txt_to_bin("tracking_map.txt","tracking_map.bin");
                        // 可以在屏幕上提示 "Save OK!"
                        printf("------save successful------\n");
                    }
                }
                else
                {
                     g_menu_instance->ips_display->show_string(1,1,"map too short");
                }
                
            }
            break;

        case 3: // 返回键
            // 重置所有状态
            ahrs.reset();
            path_tracker_component.reset();
            
            g_menu_instance->mode_inter_flag = 0;
            break;
    }
}

// 惯导路径复现
void MyMenu::tracking_by_map(uint8 cl_action){
    if (!path_tracker_component.is_reproduction) {
        g_menu_instance->ips_display->show_string(1,1,"1, reproduction");
        g_menu_instance->ips_display->show_string(1,1+1*16,"2, check_and_load_map");
        g_menu_instance->ips_display->show_string(1,1+2*16,"3, reset_status");
        g_menu_instance->ips_display->show_string(1,1+4*16,"4, return");
        g_menu_instance->ips_display->update();
    }else{
        path_tracker_component.find_closest_index();
        path_tracker_component.get_look_ahead_point(3);
        // path_tracker_component.calculate_target_yaw();
    }

    switch (cl_action)
    {
    case 0: // 开始复现
        // 开始前检查一下内存里的 map 是否为空
        if (path_tracker_component.tracking_map.empty()) {
            printf("Error: Map not loaded. Press 'Confirm' first.\n");
        } else {
            printf("Resuming path following...\n");
            ahrs.reset();
            path_tracker_component.reset();
            // 切换小车状态机到自动驾驶模式

            path_tracker_component.is_reproduction = true;
            
        }
        break;

    case 1: // 确认地图并加载地图
        printf("Checking map file...\n");
        
        // 尝试从磁盘加载二进制地图到内存
        if (path_tracker_component.load_binary_map("tracking_map.bin")) {
            // 加载成功后，打印地图信息
            printf("Map is available!\n");
            printf("Points: %d\n", (int)path_tracker_component.tracking_map.size());
            // 重置搜索起始点，确保从头开始
            path_tracker_component.last_closest_idx = 0;
             
        } else {
            // 加载失败，打印具体原因
            printf("Can't loading map");
        }
        break;

    case 2: // 重置位置，角度，路径点
        path_tracker_component.reset(); // 清除坐标累积
        path_tracker_component.last_closest_idx = 0; // 重置搜索索引
        ahrs.reset(); //解算器重置

        printf("Position and Index Reset Done.\n");
        break;

    case 3: // 返回键
        //清除所有状态
        path_tracker_component.reset();
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        break;

    default:
        break;
    }
}

// 无刷电调校准菜单
void MyMenu::brushless_calibration(uint8 cl_action)
{
    switch (cl_action)
    {
    case 0: 
        esc_set_power(100);
        break;
    case 1:
        esc_set_power(0);
        break;
    case 3: // 返回键
        g_menu_instance->mode_inter_flag = 0; // 返回菜单
        esc_set_power(0);    // 安全处理，关闭无刷电机
        break;
    default:
        break;
    }
    g_menu_instance->ips_display->show_string(1,1,"1.full power");
    g_menu_instance->ips_display->show_string(1,1+16*1,"2.min power");
    g_menu_instance->ips_display->update();

}
