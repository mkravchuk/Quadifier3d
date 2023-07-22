#include "stdafx.h"
#include "Utils_file.h"

namespace utils
{
    namespace file
    {
        bool Exists(const string& filename)
        {
            struct stat buf;
            if (stat(filename.c_str(), &buf) != -1)
            {
                return true;
            }
            return false;
        }
        string Read(const string &filename)
        {
            std::ifstream t(filename);
            return std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        };
        bool Read(const string &filename, std::vector<char>& buffer, int reserveFreeSpaceAtBegining, int reserveFreeSpaceAtEnd, char reserveChar)
        {
            std::ifstream file(filename.c_str(), std::ios::binary);
            if (!file.is_open()) return false;

            file.seekg(0, std::ios::end);
            std::streamoff filesize = file.tellg();
            file.seekg(0, std::ios::beg);

            std::streamoff buffersize = filesize + reserveFreeSpaceAtBegining + reserveFreeSpaceAtEnd;
            buffer.reserve(buffersize);
            buffer.resize(buffersize);
            file.read(&buffer[reserveFreeSpaceAtBegining], filesize);
            file.close();
            for (int i = 0; i < reserveFreeSpaceAtBegining; i++) buffer[i] = reserveChar;
            for (int i = 0; i < reserveFreeSpaceAtEnd; i++) buffer[buffersize - 1 - i] = reserveChar;
            return true;
        };
        bool Read(const string &filename, Vector<char>& buffer, int reserveFreeSpaceAtBegining, int reserveFreeSpaceAtEnd, char reserveChar)
        {
            std::ifstream file(filename.c_str(), std::ios::binary);
            if (!file.is_open()) return false;

            file.seekg(0, std::ios::end);
            std::streamoff filesize = file.tellg();
            file.seekg(0, std::ios::beg);

            std::streamoff buffersize = filesize + reserveFreeSpaceAtBegining + reserveFreeSpaceAtEnd;
            buffer.resize(buffersize);
            file.read(&buffer(reserveFreeSpaceAtBegining), filesize);
            file.close();
            for (int i = 0; i < reserveFreeSpaceAtBegining; i++) buffer(i) = reserveChar;
            for (int i = 0; i < reserveFreeSpaceAtEnd; i++) buffer(buffersize - 1 - i) = reserveChar;
            return true;
        };
        string ExtractDirName(const string& fileName)
        {
            basic_string <char>::size_type  pos = fileName.rfind("/");
            if (pos == string::npos)
            {
                pos = fileName.rfind("\\");
            }

            if (pos == string::npos)
            {
                return "";
            }
            return fileName.substr(0, pos + 1);
        }
        string ExtractFileName(const string& fileName)
        {
            basic_string <char>::size_type  pos = fileName.rfind("/");
            if (pos == string::npos)
            {
                pos = fileName.rfind("\\");
            }

            if (pos == string::npos)
            {
                return fileName;
            }
            return fileName.substr(pos + 1, fileName.size() - pos - 1);
        }

        string ExtractExtension(const string& fileName)
        {
            basic_string <char>::size_type  pos = fileName.rfind(".");

            if (pos == string::npos)
            {
                return "";
            }

            return fileName.substr(pos + 1, fileName.size() - pos - 1);
        }
        bool Write(const string& fileName, const char* text, int textSize, bool appendToFile)
        {
            std::vector<pair<const char*, int>> texts;
            texts.push_back({ text, textSize });
            return Write(fileName, texts, appendToFile);
        }
        bool Write(const string& fileName, const std::vector<pair<const char*, int>>& texts, bool appendToFile)
        {
            fstream file;
            if (appendToFile)
            {
                file.open(fileName.c_str(), ios::out | ios::binary | ios::app);
            }
            else
            {
                file.open(fileName.c_str(), ios::out | ios::binary | ios::trunc);
            }
            if (!file.good() || !file.is_open())
            { //file.eof() ||
                cerr << endl << "Error: utils::file::Write: Cannot open file " << fileName << endl << flush;
                return false;
            }
            for (const auto& text : texts)
            {
                file.write(text.first, text.second);
            }
            file.flush();
            file.close();
            return true;
        }
        bool Write(const string& fileName, const string& text, bool appendToFile)
        {
            std::vector<string> texts;
            texts.push_back(text);
            return Write(fileName, texts, appendToFile);
        }
        bool Write(const string& fileName, const std::vector<string>& texts, bool appendToFile)
        {
            fstream file;
            if (appendToFile)
            {
                file.open(fileName.c_str(), ios::out | ios::binary | ios::app);
            }
            else
            {
                file.open(fileName.c_str(), ios::out | ios::binary | ios::trunc);
            }
            if (!file.good() || !file.is_open())
            { //file.eof() ||
                cerr << endl << "Error: utils::file::Write: Cannot open file " << fileName << endl << flush;
                return false;
            }
            for (const auto& text : texts)
            {
                file.write(text.c_str(), text.size());
            }
            file.flush();
            file.close();
            return true;
        }

    }

}

