#include "stdafx.h"
#include "Utils_strings.h"
#include <cctype>

namespace utils
{
    namespace strings
    {
        bool IsInt(const string& intValue)
        {
            return extractInts(Trim(intValue)).size() > 0;
        }
        bool IsDouble(const string& doubleValue)
        {
            std::vector<int> ints;
            std::vector<D> doubles;
            int ints_added_count;
            int doubles_added_count;
            utils::strings::extractValues(doubleValue.c_str(), ints, doubles, ints_added_count, doubles_added_count);
            return doubles.size() > 0;
        }
        bool extractValues(const char* str, std::vector<int>& ints, std::vector<D>& doubles, int& ints_added_count, int& doubles_added_count)
        {
            ints_added_count = 0;
            doubles_added_count = 0;

            const char* p = str;
            bool isValuesStarted = false;
            bool isDouble = false;
            bool isExponent = false;
            bool isExponentMinus = false;
            bool isNegative = false;
            long long i = 0;
            long long d = 0;
            int dn = 0;
            int e = 0;
            const D dnm[16] = { 1, 0.1f, 0.01f, 0.001f, 0.0001f, 0.00001f, 0.000001f, 0.0000001f, 0.00000001f, 0.000000001f, 0.0000000001f, 0.00000000001f, 0.000000000001f, 0.0000000000001f, 0.00000000000001f, 0.000000000000001f };

            auto finishValue = [&]()
            {
                if (!isValuesStarted) return;

                if (isDouble)
                {
                    D r = i;
                    if (dn < 16)
                    {
                        r += dnm[dn] * d;
                    }
                    else
                    {
                        r += d / std::pow(10.0, dn);
                    }
                    if (isNegative)
                    {
                        r = -r;
                    }

                    if (isExponent)
                    {
                        D exp = std::pow(10.0, e);
                        if (isExponentMinus)
                        {
                            r /= exp;
                        }
                        else
                        {
                            r *= exp;
                        }
                    }

                    if (doubles_added_count < doubles.size())
                    {
                        doubles[doubles_added_count] = r;
                    }
                    else
                    {
                        doubles.push_back(r);
                    }
                    doubles_added_count++;
                }
                else
                {
                    if (isNegative)
                    {
                        i = -i;
                    }
                    if (ints_added_count < ints.size())
                    {
                        ints[ints_added_count] = i;
                    }
                    else
                    {
                        ints.push_back(i);
                    }
                    ints_added_count++;
                }

                i = 0;
                d = 0;
                dn = 0;
                e = 0;
                isValuesStarted = false;
                isNegative = false;
                isDouble = false;
                isExponent = false;
                isExponentMinus = false;
            };

            while (*p != 0)
            {
                switch (*p)
                {
                    case'0':
                    case'1':
                    case'2':
                    case'3':
                    case'4':
                    case'5':
                    case'6':
                    case'7':
                    case'8':
                    case'9':
                        isValuesStarted = true;
                        if (isExponent)
                        {
                            e = e * 10 + (*p - '0');
                        }
                        else if (isDouble)
                        {
                            d = d * 10 + (*p - '0');
                            dn++;
                        }
                        else
                        {
                            i = i * 10 + (*p - '0');
                        }
                        break;
                    case '.':
                        isDouble = true;
                        break;
                    case 'e':
                    case 'E':
                        isExponent = true;
                        e = 0;
                        break;
                    case '-':
                        if (isExponent)
                        {
                            isExponentMinus = true;
                        }
                        else
                        {
                            isNegative = true;
                        }
                        break;
                    default:
                        finishValue();
                        break;
                }
                p++;
            }
            finishValue();

            if (ints_added_count > ints.size()) ints.resize(ints_added_count);
            if (doubles_added_count > doubles.size()) doubles.resize(doubles_added_count);
            return ints_added_count > 0 || doubles_added_count > 0;
        }
        std::vector<int> extractInts(const string& str)
        {
            std::vector<int> ints;
            std::vector<D> doubles;
            int ints_added_count;
            int doubles_added_count;
            utils::strings::extractValues(str.c_str(), ints, doubles, ints_added_count, doubles_added_count);
            return ints;
        }

        int split(const string &str, std::vector<string> &sub_strs, const string& charDelimiters, bool removeEmpty)
        {
            size_t pos = str.find_first_of(charDelimiters);
            size_t initialPos = 0;
            sub_strs.clear();

            // Decompose statement
            while (pos != string::npos)
            {
                int length = pos - initialPos;
                if (length == 0 && removeEmpty)
                {
                    // dont add empty string
                }
                else
                {
                    sub_strs.push_back(str.substr(initialPos, length));
                }
                initialPos = pos + 1;
                pos = str.find_first_of(charDelimiters, initialPos);
            }

            // Add the last one
            int length = std::min(pos, str.size()) - initialPos;
            if (length != 0)
            {
                sub_strs.push_back(str.substr(initialPos, length));
            }

            return sub_strs.size();
        }

        int split_byChar(const string &str, std::vector<string> &sub_strs, char delimiter, bool removeEmpty)
        {
            size_t pos = str.find(delimiter);
            size_t initialPos = 0;
            sub_strs.clear();

            // Decompose statement
            while (pos != string::npos)
            {
                int length = pos - initialPos;
                if (length == 0 && removeEmpty)
                {
                    // dont add empty string
                }
                else
                {
                    sub_strs.push_back(str.substr(initialPos, length));
                }
                initialPos = pos + 1;

                pos = str.find(delimiter, initialPos);
            }

            // Add the last one
            int length = std::min(pos, str.size()) - initialPos;
            if (length != 0)
            {
                sub_strs.push_back(str.substr(initialPos, length));
            }

            return sub_strs.size();
        }  

        string ToUpper(string text)
        {            
            //transform(text.begin(), text.end(), text.begin(), static_cast<int(*)(int)>(toupper));
            transform(text.begin(), text.end(), text.begin(), [](unsigned char c)
            {
                return static_cast<char>(std::toupper(c));
            });
            return text; 
        }
        string ToLower(string text)
        {
            //transform(text.begin(), text.end(), text.begin(), static_cast<int(*)(int)>(tolower));
            transform(text.begin(), text.end(), text.begin(), [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            });
            return text;
        }
        bool StartWith(const string& text, const string& subText)
        {
            return (text.find(subText) == 0);
        }

        string TrimLeft(const string& str, const string& chars)
        {
            string strCopy = str;
            strCopy.erase(0, str.find_first_not_of(chars));
            return strCopy;
        }

        string TrimRight(const string& str, const string& chars)
        {
            string strCopy = str;
            strCopy.erase(str.find_last_not_of(chars) + 1);
            return strCopy;
        }

        string Trim(const string& str, const string& chars)
        {
            return TrimLeft(TrimRight(str, chars), chars);
        }

        string ReplaceAll(const string& str, const string& from, const string& to)
        {
            string strCopy = str;
            size_t pos = 0;
            while ((pos = strCopy.find(from, pos)) != std::string::npos)
            {
                auto len = from.length();
                strCopy.replace(pos, len, to.c_str());
                pos += to.length(); // Handles case where 'to' is a substring of 'from'
            }
            return strCopy;
        }
    }

}


