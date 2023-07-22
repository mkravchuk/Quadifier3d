#include "stdafx.h"
#include "TestAbstractClassInheritance.h"





// Does C++ really have Abstract class ???
#include <iostream>

    // Abstract class or Virtual class ???



struct ISize
{
    virtual int size() = 0; // pure virtual method  - but how to define abstract method than will not create virtual table ???
    int getSize()
    {
        return size();
    }
};

struct Size_Base
{
    int size()
    {
        return 0;
    };
    int getSize()
    {
        return size();
    }
};

struct Size_AbstractImpl : ISize
{
    int size() override
    {
        return 0;
    }
};


// Chars implements Abstract class (notice: not overrides but implements)
struct Chars_AbstractImpl : ISize
{
    char data[3];
    int size() override
    {
        return sizeof(data);
    }
};

// Chars has virtual method    
struct Chars_Virtual
{
    char data[3];
    virtual int size()
    {
        return sizeof(data);
    }
    int getSize()
    {
        return size();
    }
};

// Chars is simple class with no inheritance or virtual methods
struct Chars : public Size_Base
{
    char data[3];
    int size() 
    {
        return sizeof(data);
    }
};


void TestAbstractClassInheritance()
{
    Chars chars;
    Chars_Virtual chars_virtual;
    Chars_AbstractImpl chars_abstract_impl;
    Size_AbstractImpl size_abstract_impl;

    std::cout << "Hello C++ community!   Does C++ really have Abstract classes or just fake them using Virtual classes?   How to achieve OOP without size and performance penalty?" << std::endl;
    std::cout << std::endl;
    std::cout << "   getSize() = " << chars.getSize() << "   sizeof(Chars) = " << sizeof(chars) << "                 OK      of course getSize() != sizeof() if we use simple class as a base class" << std::endl;
    std::cout << "   getSize() = " << chars_virtual.getSize() << "   sizeof(Chars_Virtual) = " << sizeof(chars_virtual) << "        OK      of course getSize() < sizeof() if we want a virtual class" << std::endl;
    std::cout << "   getSize() = " << chars_abstract_impl.getSize() << "   sizeof(Chars_AbstractImpl) = " << sizeof(chars_abstract_impl) << "   WRONG   should be 3, because implementation of abstract class is same as merging classes" << std::endl;
    std::cout << "   getSize() = " << size_abstract_impl.getSize() << "   sizeof(Size_AbstractImpl) = " << sizeof(size_abstract_impl) << "     WRONG   should be 0, because abstract class dont own any data" << std::endl;
}