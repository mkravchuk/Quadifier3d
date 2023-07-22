#pragma once


namespace utils
{
    namespace strings
    {
        //Example:
        //vector<int> ints;
        //vector<D> doubles;
        //int ints_added_count;
        //int doubles_added_count;
        //if (utils::strings::extractValues(line, ints, doubles, ints_added_count, doubles_added_count) && doubles_added_count == 3)
        bool IsInt(const string& intValue);
        bool IsDouble(const string& doubleValue);
        bool extractValues(const char* str, std::vector<int>& ints, std::vector<D>& doubles, int& ints_added_count, int& doubles_added_count);
        std::vector<int> extractInts(const string& str);
        int split(const string &str, std::vector<string> &sub_strs, const string& charDelimiters, bool removeEmpty = true);
        int split_byChar(const string &str, std::vector<string> &sub_strs, char delimiter, bool removeEmpty = true);

        string ToUpper(string text);
        string ToLower(string text);
        bool StartWith(const string& text, const string& subText);
        template<typename ARRAYTYPE>
        void printArray(string title, ARRAYTYPE* a, int count, bool printEndOfLine = false)
        {
            cout << title << "={";
            for (int i = 0; i < count; i++)
            {
                if (i != 0) cout << ",";
                cout << a[i];
            }
            cout << "}";
            if (printEndOfLine) cout << endl;
        }
        template<typename ARRAYTYPE>
        void printArray(string title, std::vector<ARRAYTYPE> a, int count, bool printEndOfLine = false)
        {
            cout << title << "={";
            for (int i = 0; i < count; i++)
            {
                if (i != 0) cout << ",";
                cout << a[i];
            }
            cout << "}";
            if (printEndOfLine) cout << endl;
        }
        string TrimLeft(const string& str, const string& chars = "\t\n\v\f\r ");
        string TrimRight(const string& str, const string& chars = "\t\n\v\f\r ");
        string Trim(const string& str, const string& chars = "\t\n\v\f\r ");
        string ReplaceAll(const string& str, const string& from, const string& to);


    }

}
