/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/



#include "my_key.hpp"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 无
// 返回参数 无
// 使用示例 MyKey key_manager;
// 备注信息 初始化按键GPIO对象和状态信息
//-------------------------------------------------------------------------------------------------------------------
MyKey::MyKey()
{
    // 初始化GPIO对象数组
    key_gpio[KEY_UP_IDX] = new zf_driver_gpio(ZF_GPIO_KEY_1);
    key_gpio[KEY_DOWN_IDX] = new zf_driver_gpio(ZF_GPIO_KEY_2);
    key_gpio[KEY_OK_IDX] = new zf_driver_gpio(ZF_GPIO_KEY_3);
    key_gpio[KEY_BACK_IDX] = new zf_driver_gpio(ZF_GPIO_KEY_4);
    
    // 初始化按键状态信息
    for(uint8 i = 0; i < KEY_NUM; i++)
    {
        keys[i].state = KEY_RELEASED;
        keys[i].counter = 0;
        keys[i].short_press_flag = 0;
        keys[i].long_press_flag = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用
// 备注信息 释放GPIO对象资源
//-------------------------------------------------------------------------------------------------------------------
MyKey::~MyKey()
{
    for(uint8 i = 0; i < KEY_NUM; i++)
    {
        if(key_gpio[i] != nullptr)
        {
            delete key_gpio[i];
            key_gpio[i] = nullptr;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 按键扫描函数
// 参数说明 无
// 返回参数 无
// 使用示例 key_manager.scan_keys();
// 备注信息 使用状态机实现按键消抖和长短按检测，需要定时调用
//-------------------------------------------------------------------------------------------------------------------
void MyKey::scan_keys(void)
{
    for (uint8 i = 0; i < KEY_NUM; i++)
    {
        // 读取按键电平，假设按下为低电平(0)，松开为高电平(1)
        uint8 pin_level = (key_gpio[i]->get_level() == 0) ? 1 : 0;

        switch (keys[i].state)
        {
        case KEY_RELEASED:
            if (pin_level)
            {
                keys[i].state = KEY_DEBOUNCE;
                keys[i].counter = 0;
            }
            break;

        case KEY_DEBOUNCE:
            if (pin_level)
            {
                if (++keys[i].counter >= KEY_DEBOUNCE_TIME)
                {
                    keys[i].state = KEY_PRESSED;
                    keys[i].counter = 0; // 重置计数器用于长按计时
                }
            }
            else
            {
                keys[i].state = KEY_RELEASED;
            }
            break;

        case KEY_PRESSED:
            if (!pin_level)
            {
                keys[i].short_press_flag = 1; // 标记短按
                keys[i].state = KEY_RELEASED;
            }
            else if (++keys[i].counter >= KEY_LONG_PRESS_TIME)
            {
                keys[i].state = KEY_LONG_PRESS; // 进入长按状态
            }
            break;

        case KEY_LONG_PRESS:
            if (!pin_level)
            {
                keys[i].long_press_flag = 1; // 松开时才标记长按
                keys[i].state = KEY_RELEASED;
            }
            break;

        default:
            keys[i].state = KEY_RELEASED;
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取菜单动作
// 参数说明 无
// 返回参数 MenuAction 菜单动作枚举值
// 使用示例 MenuAction action = key_manager.get_menu_action();
// 备注信息 将按键状态转换为菜单动作，会自动清除按键标志位
//-------------------------------------------------------------------------------------------------------------------
MenuAction MyKey::get_menu_action(void)
{
    static const MenuAction key_mapping[KEY_NUM] = {
        [KEY_UP_IDX] = MENU_UP,
        [KEY_DOWN_IDX] = MENU_DOWN,
        [KEY_OK_IDX] = MENU_OK,
        [KEY_BACK_IDX] = MENU_BACK
    };

    // 检测长按（松开后触发）
    for (uint8 i = 0; i < KEY_NUM; i++)
    {
        if (keys[i].long_press_flag)
        {
            MenuAction action = (MenuAction)(key_mapping[i] + 5); // 长按动作=原值+5
            keys[i].long_press_flag = 0; // 清除标志位
            return action;
        }
    }

    // 检测短按（自动清除标志）
    for (uint8 i = 0; i < KEY_NUM; i++)
    {
        if (keys[i].short_press_flag)
        {
            keys[i].short_press_flag = 0;
            return key_mapping[i];
        }
    }

    return WAITING;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 清除指定按键的长按标志
// 参数说明 key_id 按键索引
// 返回参数 无
// 使用示例 key_manager.clear_long_press_flag(KEY_UP_IDX);
// 备注信息 手动清除特定按键的长按标志位
//-------------------------------------------------------------------------------------------------------------------
void MyKey::clear_long_press_flag(uint8 key_id)
{
    if (key_id < KEY_NUM)
    {
        keys[key_id].long_press_flag = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 清除所有按键的长按标志
// 参数说明 无
// 返回参数 无
// 使用示例 key_manager.clear_all_long_press_flags();
// 备注信息 清除所有按键的长按标志位
//-------------------------------------------------------------------------------------------------------------------
void MyKey::clear_all_long_press_flags(void)
{
    for (uint8 i = 0; i < KEY_NUM; i++)
    {
        keys[i].long_press_flag = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取按键状态信息
// 参数说明 key_id 按键索引
// 返回参数 KeyInfo* 按键状态信息指针
// 使用示例 KeyInfo* info = key_manager.get_key_info(KEY_UP_IDX);
// 备注信息 获取指定按键的详细状态信息
//-------------------------------------------------------------------------------------------------------------------
KeyInfo* MyKey::get_key_info(uint8 key_id)
{
    if (key_id < KEY_NUM)
    {
        return &keys[key_id];
    }
    return nullptr;
}