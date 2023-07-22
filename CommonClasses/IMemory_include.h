
#pragma region Read/Write (single value)
CommonClassesINLINE T& operator()(Index index)
{
    #if DEBUG
    assert(index >= 0 && index < size() && "index is out of range");
    #endif
    return data()[index];
}
CommonClassesINLINE T operator()(Index index) const
{
    #if DEBUG
    assert(index >= 0 && index < size() && "index is out of range");
    #endif
    return data()[index];
}
CommonClassesINLINE T& operator[](Index index)
{
    #if DEBUG
    assert(index >= 0 && index < size() && "index is out of range");
    #endif
    return data()[index];
}
CommonClassesINLINE T operator[](Index index) const
{
    #if DEBUG
    assert(index >= 0 && index < size() && "index is out of range");
    #endif
    return data()[index];
}
#pragma endregion 

#pragma region  Fill data
void setZero()
{
    for (Index i = 0; i < size(); i++)
    {
        data()[i] = 0;
    }
}
void setConstant(T value)
{
    for (Index i = 0; i < size(); i++)
    {
        data()[i] = value;
    }
}
void setRandom()
{
    for (Index i = 0; i < size(); i++)
    {
        data()[i] = static_cast<T>(rand());
    }
}
#pragma endregion 

#pragma region Agregators
T sum() const
{
    if (size() == 2)
    {
        return data()[0] + data()[1];
    }
    else if (size() == 3)
    {
        return data()[0] + data()[1] + data()[2];
    }
    else if (size() == 4)
    {
        return data()[0] + data()[1] + data()[2] + data()[3];
    }
    else
    {
        T res = 0;
        for (Index i = 0; i < size(); i++) res += data()[i];
        return res;
    }
}
T minCoeff() const
{
    T res = T();
    if (size() > 0) res = data()[0];
    for (Index i = 1; i < size(); i++) if (data()[i] < res) res = data()[i];
    return res;
}
T maxCoeff() const
{
    T res = T();
    if (size() > 0) res = data()[0];
    for (Index i = 1; i < size(); i++) if (data()[i] > res) res = data()[i];
    return res;
}
void minCoeff(Index& index) const
{
    index = 0;
    T res = T();
    if (size() > 0) res = data()[0];
    for (Index i = 1; i < size(); i++) if (data()[i] < res)
    {
        res = data()[i];
        index = i;
    }
}
void maxCoeff(Index& index) const
{
    index = 0;
    T res = T();
    if (size() > 0) res = data()[0];
    for (Index i = 1; i < size(); i++) if (data()[i] > res)
    {
        res = data()[i];
        index = i;
    }
}
#pragma endregion 

#pragma region Convertions
std::string toString() const
{
    std::string s = "";
    for (Index i = 0; i < size(); i++)
    {
        if (i != 0) s += ", ";
        s += std::to_string(data()[i]);
    }
    return s;
}
#pragma endregion 
