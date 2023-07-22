#include "stdafx.h"
#include "Utils_cursor.h"
#include <wtypes.h>


HCURSOR cursorWait;
HCURSOR cursorCross;
HCURSOR cursorArrow;
vector<HCURSOR> cursors_stack;


namespace utils
{
    namespace cursor
    {
        mutex _mtx;

        void InitCursors()
        {
            if (cursorWait == NULL)
            {
                cursorWait = LoadCursor(NULL, IDC_WAIT);
                cursorCross = LoadCursor(NULL, IDC_CROSS);
                cursorArrow = LoadCursor(NULL, IDC_ARROW);

                // Create a custom cursor based on a resource.  
                // https://docs.microsoft.com/en-us/windows/desktop/menurc/using-cursors
                //HINSTANCE hinst;            // handle to current instance 
                //hCurs2 = LoadCursor(hinst, MAKEINTRESOURCE(240));
            }
        }
        void SetCursor(HCURSOR cursor, bool _on)
        {
            _mtx.lock(); // thread safe
            InitCursors();
            if (_on)
            {
                cursors_stack.push_back(cursor);
                SetCursor(cursor);
            }
            else
            {
                if (cursors_stack.size() != 0)
                {
                    cursors_stack.pop_back();
                }
                if (cursors_stack.size() == 0)
                {
                    SetCursor(cursorArrow);
                }
                else
                {
                    SetCursor(cursors_stack.back());
                }
            }
            _mtx.unlock(); // thread safe
        }

        HCURSOR toCursor(CursorType cursorType)
        {
            switch (cursorType)
            {
                case CursorType::Wait:
                   return cursorWait;
                case CursorType::Cross:
                    return cursorCross;
                default:
                    return cursorArrow;
            }
        }
        void SetCursor(CursorType cursorType, bool _on)
        {
            SetCursor(toCursor(cursorType), _on);
        }

        void Set(CursorType cursorType)
        {
            SetCursor(toCursor(cursorType), true);
        }

        void Cancel(CursorType cursorType)
        {
            SetCursor(toCursor(cursorType), false);
        }

    }
}