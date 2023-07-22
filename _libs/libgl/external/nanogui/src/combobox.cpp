/*
    src/combobox.cpp -- simple combo box widget based on a popup button

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/combobox.h>
#include <nanogui/layout.h>
#include <nanogui/serializer/core.h>
#include <nanogui/label.h>
#include <cassert>

NAMESPACE_BEGIN(nanogui)

ComboBox::ComboBox(Widget *parent) : PopupButton(parent), mSelectedIndex(0) {
}

ComboBox::ComboBox(Widget *parent, const std::vector<std::string> &items)
    : PopupButton(parent), mSelectedIndex(0) {
    setItems(items);
}

ComboBox::ComboBox(Widget *parent, const std::vector<std::string> &items, const std::vector<std::string> &itemsShort)
    : PopupButton(parent), mSelectedIndex(0) {
    setItems(items, itemsShort);
}

void ComboBox::setSelectedIndex(int idx) {
    if (mItemsShort.empty())
        return;

    // v0 - origin
    //const std::vector<Widget *> &children = popup()->children();
    //((Button *) children[mSelectedIndex])->setPushed(false);
    //((Button *) children[idx])->setPushed(true);

    // v1 - supported GridLayout
    for (Widget * w : popup()->children())
    {
        Button * b = dynamic_cast<Button*>(w);
        if (b == nullptr) continue;
        b->setPushed(b->index() == idx);
    }

    mSelectedIndex = idx;
    setCaption(mItemsShort[idx]);
    markDity();
}

ComboBox* ComboBox::setItems(const std::vector<std::string> &items, const std::vector<std::string> &itemsShort) {
    markDity();
    assert(items.size() == itemsShort.size());
    //mItems = items;
    //mItemsShort = itemsShort;
    if (mSelectedIndex < 0 || mSelectedIndex >= (int) items.size())
        mSelectedIndex = 0;
    while (mPopup->childCount() != 0)
        mPopup->removeChild(mPopup->childCount()-1);

    // calculate columnsCount
    int columnsCount = 0;
    for(int i = 0; i < items.size(); i++)
    {
        const std::string &str = items[i];
        if (str.find("#") == 0)
        {
            columnsCount++;
        }
        else if (columnsCount == 0)
        {
            columnsCount++;
        }
    }

    // copy items
    mItems.clear();
    mItemsShort.clear();
    for (int i = 0; i < items.size(); i++)
    {
        if (items[i].find("#") != 0)
        {
            mItems.push_back(items[i]);
            mItemsShort.push_back(itemsShort[i]);
        }
    }

    // set layout
    if (columnsCount > 1)
    {
        auto gridLayout = new GridLayout(nanogui::Orientation::Horizontal, columnsCount, Alignment::Fill, 15, 6);
        mPopup->setLayout(gridLayout);
    }
    else
    {
        auto gridLayout = new GroupLayout(10);
        mPopup->setLayout(gridLayout);
    }

    // add buttons
    int gridLayoutResolutionIndex = -1;
    int index = 0;
    for (const auto &str: items) 
    {
        if (str.find("#") == 0)
        {
            gridLayoutResolutionIndex++;
            Label* label = new Label(mPopup, str.substr(1, str.size() - 1) , "sans-bold", 20);
            if (columnsCount > 1)
            {
                label->setGridLayoutResolutionIndex(gridLayoutResolutionIndex);
            }
            continue;
        }
        else if (gridLayoutResolutionIndex == -1)
        {
            gridLayoutResolutionIndex = 0;
        }

        Button *button = new Button(mPopup, str);
        button->setIndex(index);
        button->setFlags(Button::RadioButton);
        if (columnsCount > 1)
        {
            button->setGridLayoutResolutionIndex(gridLayoutResolutionIndex);
        }
        button->setCallback([&, index] {
            mSelectedIndex = index;
            setCaption(mItemsShort[index]);
            setPushed(false);
            popup()->setVisible(false);
            if (mCallback)
                mCallback(index);
        });

        index++;
    }
    setSelectedIndex(mSelectedIndex);
    return this;
}

bool ComboBox::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (rel.y() < 0) {
        setSelectedIndex(std::min(mSelectedIndex+1, (int)(items().size()-1)));
        if (mCallback)
            mCallback(mSelectedIndex);
        return true;
    } else if (rel.y() > 0) {
        setSelectedIndex(std::max(mSelectedIndex-1, 0));
        if (mCallback)
            mCallback(mSelectedIndex);
        return true;
    }
    return Widget::scrollEvent(p, rel);
}

void ComboBox::save(Serializer &s) const {
    Widget::save(s);
    s.set("items", mItems);
    s.set("itemsShort", mItemsShort);
    s.set("selectedIndex", mSelectedIndex);
}

bool ComboBox::load(Serializer &s) {
    markDity();
    if (!Widget::load(s)) return false;
    if (!s.get("items", mItems)) return false;
    if (!s.get("itemsShort", mItemsShort)) return false;
    if (!s.get("selectedIndex", mSelectedIndex)) return false;
    return true;
}

NAMESPACE_END(nanogui)
