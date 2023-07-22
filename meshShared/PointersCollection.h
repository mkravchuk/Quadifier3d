#pragma once

template <class T>
class PointersCollection
{
public:
    vector<T*> Objects;

    T* Add()
    {
        Objects.push_back(new T());
        return Objects.back();
    }

    void Add(T* obj)
    {
        Objects.push_back(obj);
    }

    bool Remove(const T* obj)
    {
        auto index = std::find(Objects.begin(), Objects.end(), obj);
        if (index != Objects.end())
        {
            delete obj;
            Objects.erase(index);
            return true;
        }
        return false;
    }

    void Clear()
    {
        for (auto obj : Objects)
        {
            delete obj;
        }
        Objects.clear();
    }

    PointersCollection()
    {
    }

    ~PointersCollection() // destructor (Rule of five)
    {
        Clear();
    }

    PointersCollection(const PointersCollection& o)  // copy constructor (Rule of five)
    {
        Objects.reserve(o.Objects.size());
        for (int i = 0; i < o.Objects.size(); i++)
        {
            Objects.push_back(new T(o.Objects[i]));
        }
    }

    PointersCollection(PointersCollection&& o) noexcept // move constructor (Rule of five)
        : Objects(std::move(o.Objects))
    {
    }

    PointersCollection& operator=(const PointersCollection& other) // copy assignment (Rule of five)
    {
        return *this = PointersCollection(other);
    }

    PointersCollection& operator=(PointersCollection&& other) noexcept // move assignment (Rule of five)
    {
        std::swap(Objects, other.Objects);
        return *this;
    }
};

