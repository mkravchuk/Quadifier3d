#pragma once


namespace utils
{
    namespace file
    {
        bool Exists(const string& filename);
        string Read(const string &filename);
        bool Read(const string &filename, std::vector<char>& buffer, int reserveFreeSpaceAtBegining = 0, int reserveFreeSpaceAtEnd = 0, char reserveChar = ' ');
        bool Read(const string &filename, Vector<char>& buffer, int reserveFreeSpaceAtBegining = 0, int reserveFreeSpaceAtEnd = 0, char reserveChar = ' ');
        string ExtractDirName(const string& fileName);
        string ExtractFileName(const string& fileName);
        string ExtractExtension(const string& filename);
        bool Write(const string& fileName, const char* text, int textSize, bool appendToFile = false);
        bool Write(const string& fileName, const std::vector<pair<const char*, int>>& texts, bool appendToFile = false);
        bool Write(const string& fileName, const string& text, bool appendToFile = false);
        bool Write(const string& fileName, const std::vector<string>& texts, bool appendToFile = false);
    }

}

