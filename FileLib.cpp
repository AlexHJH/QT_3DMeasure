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


/////ɾ���ļ��м��ļ�
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
@param File_Directory Ϊ�ļ���Ŀ¼
@param FileType Ϊ��Ҫ���ҵ��ļ�����
@param FilesName Ϊ����ļ���������
*/
void getFilesName(string &File_Directory, string &FileType, vector<string>&FilesName)
{
	string buffer = File_Directory + "\\*" + FileType;

	_finddata_t c_file;                             // ����ļ����Ľṹ��

	intptr_t hFile;
	hFile = _findfirst(buffer.c_str(), &c_file);   //�ҵ�һ���ļ���

	if (hFile == -1L)                              // ����ļ���Ŀ¼�´�����Ҫ���ҵ��ļ�
		printf("No files in current directory!\n", FileType);
	else
	{
		string fullFilePath;
		do
		{
			fullFilePath.clear();
			fullFilePath = File_Directory + "\\" + c_file.name;
			FilesName.push_back(fullFilePath);
		} while (_findnext(hFile, &c_file) == 0);  //����ҵ��¸��ļ������ֳɹ��Ļ��ͷ���0,���򷵻�-1  
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
		if ((_A_SUBDIR == filefind.attrib)) //��Ŀ¼
		{
			FilesName.push_back(filefind.name);
			curr = File_Directory + "\\" + filefind.name;
			filenum += 1;
		}
		else//����Ŀ¼�����ļ�     
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
