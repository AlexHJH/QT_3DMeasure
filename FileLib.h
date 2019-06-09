#ifndef FileLib_h
#define FileLib_h


#include <iostream>
#include <fstream>
#include <io.h>
#include <string>
#include <vector>

#include <QString>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>

using namespace std;

std::string & std_string_format(std::string & _str, const char * _Format, ...);
void getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName);
int visitDirectory(string File_Directory, vector<string>&FilesName);
bool DeleteDirectory(const QString &path);
void GetAllFileFolder(QString dirPath, std::vector<QString> &folder);
#endif // FileLib_h
