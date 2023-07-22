#pragma once


namespace utils
{
    namespace cursor
    {
        enum class CursorType
        {
            Default, Wait, Cross
        };
        void SetCursor(CursorType cursorType, bool _on);
        void Set(CursorType cursorType);
        void Cancel(CursorType cursorType);
    }
}