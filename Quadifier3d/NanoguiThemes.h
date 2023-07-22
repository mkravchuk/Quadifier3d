#pragma once
#include <nanogui/theme.h>
using namespace nanogui;

class NanoguiTheme : public nanogui::Theme {
public:
    enum class NanoguiThemeColorStyle {
        LightBlue, Silver, Dark
    };
    vector<string> NanoguiThemeColorStyleStr = { "Light blue", "Silver", "Dark" };

    NanoguiTheme(NVGcontext *ctx);

    // Colors
    Vector4f background_color;
    Vector4f wireframe_color;

    NanoguiThemeColorStyle ColorStyle;
    void SetColorStyle(NanoguiThemeColorStyle style);
private:
    void Refresh();
};



inline NanoguiTheme::NanoguiTheme(NVGcontext* ctx) : Theme(ctx)
{
    mStandardFontSize = 20;
    SetColorStyle(NanoguiThemeColorStyle::Dark);
}

inline void NanoguiTheme::SetColorStyle(NanoguiThemeColorStyle style)
{
    //11  22 33 44 55 66 77
    ColorStyle = style;
    if (ColorStyle == NanoguiThemeColorStyle::Dark)
    {
        background_color << 0.3f, 0.3f, 0.5f, 1.0f;
        wireframe_color << 0.0f, 0.0f, 0.0f, 1.0f;

        mTransparent = Color(0, 0);
        mBorderDark = Color(29, 255);
        mBorderLight = Color(92, 255);
        mBorderMedium = Color(35, 255);

        mTextColor = Color(255, 160);
        mDisabledTextColor = Color(255, 80);
        mTextColorShadow = Color(0, 160);
        mIconColor = mTextColor;

        mButtonGradientTopFocused = Color(64, 255);
        mButtonGradientBotFocused = Color(48, 255);
        mButtonGradientTopUnfocused = Color(74, 255);
        mButtonGradientBotUnfocused = Color(58, 255);
        mButtonGradientTopPushed = Color(41, 255);
        mButtonGradientBotPushed = Color(29, 255);

        /* Window-related */
        mWindowFillUnfocused = Color(43, 230);
        mWindowFillFocused = Color(45, 230);
        mWindowTitleUnfocused = Color(220, 160);
        mWindowTitleFocused = Color(255, 190);
        mDropShadow = Color(0, 128); // window shadow
        mWindowDropShadowSize = 10;
        mWindowCornerRadius = 2;

        mWindowHeaderGradientTop = mButtonGradientTopUnfocused;
        mWindowHeaderGradientBot = mButtonGradientBotUnfocused;
        mWindowHeaderSepTop = mBorderLight;
        mWindowHeaderSepBot = mBorderDark;

        mWindowPopup = Color(50, 255);
        mWindowPopupTransparent = Color(50, 0);
    }
    if (ColorStyle == NanoguiThemeColorStyle::LightBlue)
    {
        background_color << 1.0f, 1.0f, 1.0f, 1.0f;
        wireframe_color << 0.0f, 0.0f, 0.0f, 1.0f;

        mTransparent = Color(0,0);
        mBorderDark = Color(180, 205, 217, 255);
        mBorderLight = mTransparent;
        mBorderMedium = mTransparent;

        mTextColor = Color(0, 210);
        mDisabledTextColor = Color(0, 80);
        mTextColorShadow = Color(205, 210);
        mIconColor = Color(72, 93, 148, 255);

        mButtonGradientTopFocused = Color(237, 248, 248, 235);
        mButtonGradientBotFocused = Color(184, 227, 242, 235);
        mButtonGradientTopUnfocused = Color(244, 248, 248, 235);
        mButtonGradientBotUnfocused = Color(230, 237, 242, 235);
        mButtonGradientTopPushed = Color(167, 183, 192, 255);
        mButtonGradientBotPushed = Color(114, 160, 186, 255);


        /* Window-related */
        mWindowFillUnfocused = Color(239, 240, 243, 230);
        mWindowFillFocused =  mWindowFillUnfocused;
        mWindowTitleUnfocused = Color(85, 160);
        mWindowTitleFocused = Color(30, 190);
        mDropShadow = Color(162, 170, 184, 245); // window shadow
        mWindowDropShadowSize = 7;
        mWindowCornerRadius = 1;

        mWindowHeaderGradientTop = mButtonGradientTopUnfocused;
        mWindowHeaderGradientBot = mButtonGradientBotUnfocused;
        mWindowHeaderSepTop = mBorderLight;
        mWindowHeaderSepBot = mBorderDark;

        mWindowPopup = Color(229, 230, 233, 255);
        mWindowPopupTransparent = Color(155,0,0, 0);
    }
    if (ColorStyle == NanoguiThemeColorStyle::Silver)
    {
        background_color << 1.0f, 1.0f, 1.0f, 1.0f;
        wireframe_color << 0.0f, 0.0f, 0.0f, 1.0f;

        mTransparent = Color(0, 0);
        mBorderDark = Color(189, 189, 189, 255);
        mBorderLight = mTransparent;
        mBorderMedium = mTransparent;

        mTextColor = Color(0, 190);
        mDisabledTextColor = Color(0, 80);
        mTextColorShadow = Color(165, 0);
        mIconColor = mTextColor;

        mButtonGradientTopFocused = Color(244, 248, 248, 235);
        mButtonGradientBotFocused = Color(230, 237, 242, 235);
        mButtonGradientTopUnfocused = Color(242, 242, 242, 235);
        mButtonGradientBotUnfocused = Color(225, 225, 225, 235);
        mButtonGradientTopPushed = Color(167, 183, 192, 255);
        mButtonGradientBotPushed = Color(114, 160, 186, 255);


        /* Window-related */
        mWindowFillUnfocused = Color(240, 240, 242, 230);
        mWindowFillFocused = mWindowFillUnfocused;
        mWindowTitleUnfocused = Color(85, 160);
        mWindowTitleFocused = Color(30, 190);
        mDropShadow = Color(162, 168, 174, 235); // window shadow
        mWindowDropShadowSize = 3;
        mWindowCornerRadius = 1;

        mWindowHeaderGradientTop = mButtonGradientTopUnfocused;
        mWindowHeaderGradientBot = mButtonGradientBotUnfocused;
        mWindowHeaderSepTop = mBorderLight;
        mWindowHeaderSepBot = mBorderDark;

        mWindowPopup = Color(229, 230, 233, 255);
        mWindowPopupTransparent = Color(155, 0, 0, 0);
    }

    //mDropShadow = Color(255,0,0, 255);
    //mDropShadow = Color(162, 170, 184, 245); // window shadow
    //mDropShadow = Color(120, 122, 130, 255); // window shadow
    //mDropShadow *= 0.5;
    //mDropShadow(1) *= 0.5;
    //mDropShadow(2) *= 0.5;
    //mDropShadow(3) *= 0.5;
    //mDropShadow = Color(0, 0);

    Refresh();
}

inline void NanoguiTheme::Refresh()
{

}


