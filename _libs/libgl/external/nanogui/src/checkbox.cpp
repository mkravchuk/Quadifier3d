/*
    src/checkbox.cpp -- Two-state check box widget

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/checkbox.h>
#include <nanogui/opengl.h>
#include <nanogui/theme.h>
#include <nanogui/entypo.h>
#include <nanogui/serializer/core.h>

NAMESPACE_BEGIN(nanogui)

CheckBox::CheckBox(Widget *parent, const std::string &caption, const std::function<void(bool) > &callback)
    : Widget(parent), mCaption(caption), mPushed(false), mChecked(false),
      mCallback(callback)
{
    
}

bool CheckBox::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) 
{    
    Widget::mouseButtonEvent(p, button, down, modifiers);
    if (!mEnabled)
        return false;

    if (button == GLFW_MOUSE_BUTTON_1) {
        markDity();
        if (down) 
        {
            mPushed = true;
        } 
        else if (mPushed) 
        {
            mPushed = false;
            if (contains(p)) 
            {
                mChecked = !mChecked;
                if (mCallback)
                    mCallback(mChecked);
            }
        }
        return true;
    }
    return false;
}

Vector2i CheckBox::preferredSize(NVGcontext *ctx) const {
    if (mFixedSize != Vector2i::Zero())
        return mFixedSize;
    nvgFontSize(ctx, fontSize());
    nvgFontFace(ctx, "sans");
    return Vector2i(
        nvgTextBounds(ctx, 0, 0, mCaption.c_str(), nullptr, nullptr) +
            1.7f * fontSize(),
        fontSize() * 1.3f);
}

void CheckBox::draw(NVGcontext *ctx, bool drawOnlyGui) {
    Widget::draw(ctx, drawOnlyGui);
    
    nvgFontSize(ctx, fontSize());
    nvgFontFace(ctx, "sans");
    nvgFillColor(ctx, mEnabled ? mTheme->mTextColor : mTheme->mDisabledTextColor);
    nvgTextAlign(ctx, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);

    //if (mPushed)
    //{
    //    mPushed = mPushed;
    //}
    
    nvgText(ctx, mPos.x() + 1.2f * mSize.y() + 5, mPos.y() + mSize.y() * 0.5f,
            mCaption.c_str(), nullptr);

    int checkHeight = mSize.y();
    if (checkHeight > 19) checkHeight = 19;
    int checkY = mPos.y() + (mSize.y() - checkHeight) / 2;
   
    NVGpaint bg = nvgBoxGradient(ctx, mPos.x() + 1.5f, checkY + 1.5f,
        checkHeight - 2.0f, checkHeight - 2.0f, 3, 3,
                                 mPushed ? Color(0, 70) : Color(0, 32),
                                 Color(0, 0, 0, 180));

    nvgBeginPath(ctx);
    nvgRoundedRect(ctx, mPos.x() + 1.0f, checkY + 1.0f, checkHeight - 2.0f, checkHeight - 2.0f, 3);
    nvgFillPaint(ctx, bg);
    nvgFill(ctx);

    if (mChecked) {
        nvgFontSize(ctx, 1.5 * checkHeight);
        nvgFontFace(ctx, "icons");
        nvgFillColor(ctx, mEnabled ? mTheme->mIconColor : mTheme->mDisabledTextColor);
        nvgTextAlign(ctx, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(ctx, mPos.x() + checkHeight * 0.5f, checkY + checkHeight * 0.5f, utf8(ENTYPO_ICON_CHECK).data(), nullptr);
    }
}

void CheckBox::save(Serializer &s) const {
    Widget::save(s);
    s.set("caption", mCaption);
    s.set("pushed", mPushed);
    s.set("checked", mChecked);
}

bool CheckBox::load(Serializer &s) {
    markDity();
    if (!Widget::load(s)) return false;
    if (!s.get("caption", mCaption)) return false;
    if (!s.get("pushed", mPushed)) return false;
    if (!s.get("checked", mChecked)) return false;
    return true;
}

NAMESPACE_END(nanogui)
