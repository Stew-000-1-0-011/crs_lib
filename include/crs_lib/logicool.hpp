/*
XInputモードにしか対応していない。
DirectInputモードやその切り替えを十分に調べたら実装するかも。
まあ、コントローラーが保持する状態はx/Dモードのどちらかなのか以外にはない(はず)なので、間違えて切り替えてもきっと大丈夫。

joyパッケージをみたら、トリガーが最初0を吐く問題はノードを起動する際に渡す引数で対処可能(かもしれない)らしいことがわかった。
CRSLib::JoyToKEyButtonのon_updateは消すべきかもしれない。
キーが押された次の瞬間のみ取得、とかなら

まあせっかく実装したんだし使うか...
*/


#pragma once

#include <cstdint>

#include <sensor_msgs/Joy.h>

#include "joy_to_key_button.hpp"

namespace CRSLib
{
    struct LogicoolXInputKeyMap final
    {
        struct Axes final
        {
            enum Enum : std::uint8_t
            {
                l_stick_LR = 0,
                l_stick_UD,
                l_trigger,
                r_stick_LR,
                r_stick_UD,
                r_trigger,
                cross_LR,
                cross_UD,

                N
            };
        };

        struct Buttons final
        {
            enum Enum : std::uint8_t
            {
                a = 0,
                b,
                x,
                y,
                lb,
                rb,
                back,
                start,
                l_push,
                r_push,

                N
            };
        };
    };

    using Logicool = JoyToKeyButton<LogicoolXInputKeyMap>;
}