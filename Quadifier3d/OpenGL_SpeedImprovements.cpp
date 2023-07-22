#include "stdafx.h"
#include "OpenGL_SpeedImprovements.h"


// preinit long opengl call   https://hero.handmade.network/forums/code-discussion/t/2503-%5Bday_235%5D_opengls_pixel_format_takes_a_long_time
DWORD WINAPI _glPreinit_PixelFormat_calls(LPVOID param)
{
    HDC dc = GetDC(NULL);
    DescribePixelFormat(dc, 0, 0, NULL);
    ReleaseDC(NULL, dc);
    return 0;
}

void glPreinit_PixelFormat_calls()
{
    // call method in thread to make this work in parallel
    CreateThread(NULL, 3 * 64 * 1024, &_glPreinit_PixelFormat_calls, NULL, STACK_SIZE_PARAM_IS_A_RESERVATION, NULL); //The call as it was before could have potentially caused paging to be involved in the stack access(depending on the linker settings).It also turned out that the loading process actually required a bit more than one memory page(usually 64kb), so I made it a flat two pages.
}







static GLuint _glewStrLen(const GLubyte* s)
{
    GLuint i = 0;
    if (s == NULL) return 0;
    while (s[i] != '\0') i++;
    return i;
}
static GLuint _glewStrCLen(const GLubyte* s, GLubyte c)
{
    GLuint i = 0;
    if (s == NULL) return 0;
    while (s[i] != '\0' && s[i] != c) i++;
    return (s[i] == '\0' || s[i] == c) ? i : 0;
}
static GLboolean _glewStrSame(const GLubyte* a, const GLubyte* b, GLuint n)
{
    GLuint i = 0;
    if (a == NULL || b == NULL)
        return (a == NULL && b == NULL && n == 0) ? GL_TRUE : GL_FALSE;
    while (i < n && a[i] != '\0' && b[i] != '\0' && a[i] == b[i]) i++;
    return i == n ? GL_TRUE : GL_FALSE;
}


int _glewSearchExtension_cache_size = 0;
#define _glewSearchExtension_cache_capacity 3
GLubyte* _glewSearchExtension_cache_extensions_start[_glewSearchExtension_cache_capacity];
GLuint* _glewSearchExtension_cache_extensions_length[_glewSearchExtension_cache_capacity];
static GLuint* _glewSearchExtension_get_lengths_from_cache(const GLubyte *start, const GLubyte *end)
{
    GLuint* cachedLengths = NULL;
    // try to find lengths in cache
    for (int i = 0; i < _glewSearchExtension_cache_size; i++)
    {
        if (_glewSearchExtension_cache_extensions_start[i] == start)
        {
            cachedLengths = _glewSearchExtension_cache_extensions_length[i];
            break;
        }
    }
    // if we didnt find lengths in cache then create this cache
    if (cachedLengths == NULL && _glewSearchExtension_cache_size < _glewSearchExtension_cache_capacity)
    {
        // detect needed memory size
        const GLubyte * p = start;
        int extensions_size = 0;
        while (p < end)
        {
            GLuint len = _glewStrCLen(p, ' ');
            p += len + 1;
            extensions_size++;
        }

        // allocate memory
        //cachedLengths = new GLuint[extensions_size];
        cachedLengths = (GLuint *)malloc(extensions_size * sizeof(GLuint));

        // populate lengths in memory
        p = start;
        GLuint* nextLength = cachedLengths;
        while (p < end)
        {
            GLuint len = _glewStrCLen(p, ' ');
            p += len + 1;
            *nextLength = len;
            nextLength++;
        }

        // cache our result
        _glewSearchExtension_cache_extensions_length[_glewSearchExtension_cache_size] = cachedLengths;
        _glewSearchExtension_cache_size++;
    }

    // return lengths (if found of course)
    return cachedLengths;
}

static GLboolean _glewSearchExtension(const char* name, const GLubyte *start, const GLubyte *end)
{
    const GLubyte* p = start;
    GLuint len = _glewStrLen((const GLubyte*)name);

    GLuint* cachedLengths = _glewSearchExtension_get_lengths_from_cache(start, end);
    if (cachedLengths != NULL)
    {
        GLuint* nextLength = cachedLengths;
        while (p < end)
        {
            GLuint n = *nextLength;
            nextLength++;
            if (len == n && _glewStrSame((const GLubyte*)name, p, n)) return GL_TRUE;
            p += n + 1;
        }
    }
    else
    {
        while (p < end)
        {
            GLuint n = _glewStrCLen(p, ' ');
            if (len == n && _glewStrSame((const GLubyte*)name, p, n)) return GL_TRUE;
            p += n + 1;
        }
    }    

    return GL_FALSE;
}