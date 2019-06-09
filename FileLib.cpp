#include "FileLib.h"

std::string & std_string_format(std::string & _str, const char * _Format, ...) {
	va_list marker = NULL;
	va_start(marker, _Format);

	int num_of_chars = _vscprintf(_Format, marker);

	if (num_of_chars > _str.capacity()) {
		_str.resize(num_of_chars + 1);
	}

	vsprintf((char *)_str.c_str(), _Format, marker);

	va_end(marker);
	return _str;
}


/////删除文件夹及文件
bool DeleteDirectory(const QString &path)
{
	if (path.isEmpty())
	{
		return false;
	}
	QDir dir(path);
	if (!dir.exists())
	{
		return true;
	}

	dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
	QFileInfoList fileList = dir.entryInfoList();
	foreach(QFileInfo fi, fileList)
	{
		if (fi.isFile())
		{
			fi.dir().remove(fi.fileName());
		}
		else
		{
			DeleteDirectory(fi.absoluteFilePath());
		}
	}
	return  dir.rmpath(dir.absolutePath());
}


/*
@param File_Directory 为文件夹目录
@param FileType 为需要查找的文件类型
@param FilesName 为存放文件名的容器
*/
void getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName)
{
	string buffer = File_Directory + "\\*" + FileType;

	_finddata_t c_file;                             // 存放文件名的结构体

	intptr_t hFile;
	hFile = _findfirst(buffer.c_str(), &c_file);   //找第一个文件名

	if (hFile == -1L)                              // 检查文件夹目录下存在需要查找的文件
		printf("No files in current directory!\n", FileType);
	else
	{
		string fullFilePath;
		do
		{
			fullFilePath.clear();
			fullFilePath = File_Directory + "\\" + c_file.name;
			FilesName.push_back(fullFilePath);
		} while (_findnext(hFile, &c_file) == 0);  //如果找到下个文件的名字成功的话就返回0,否则返回-1  
		_findclose(hFile);
	}
}

int visitDirectory(string File_Directory, vector<string>&FilesName)
{
	struct _finddata_t   filefind;
	string  curr = File_Directory + "\\*.*";
	intptr_t   done = 0, i, handle;
	int filenum = 0;
	if ((handle = _findfirst(curr.c_str(), &filefind)) == -1)return -1;
	while (!(done = _findnext(handle, &filefind)))
	{
		if (!strcmp(filefind.name, "..")) {
			continue;
		}
		if ((_A_SUBDIR == filefind.attrib)) //是目录
		{
			FilesName.push_back(filefind.name);
			curr = File_Directory + "\\" + filefind.name;
			filenum += 1;
		}
		else//不是目录，是文件     
		{
		}
	}
	_findclose(handle);
	return filenum;
}



void GetAllFileFolder(QString dirPath, std::vector<QString> &folder)
{

	QDir dir(dirPath);

	dir.setFilter(QDir::Dirs);

	foreach(QFileInfo fullDir, dir.entryInfoList())
	{
		if (fullDir.fileName() == "." || fullDir.fileName() == "..") 
			continue;
		folder.push_back(fullDir.fileName());
	}

}
