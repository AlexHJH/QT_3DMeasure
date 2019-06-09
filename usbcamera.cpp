#include <windows.h>
#include "CameraApi.h"
#include "usbcamera.h"

#define VI_MAX_CAMERAS 20
DEFINE_GUID(CLSID_SystemDeviceEnum, 0x62be5d10, 0x60eb, 0x11d0, 0xbd, 0x3b, 0x00, 0xa0, 0xc9, 0x11, 0xce, 0x86);
DEFINE_GUID(CLSID_VideoInputDeviceCategory, 0x860bb310, 0x5d01, 0x11d0, 0xbd, 0x3b, 0x00, 0xa0, 0xc9, 0x11, 0xce, 0x86);
DEFINE_GUID(IID_ICreateDevEnum, 0x29840822, 0x5b84, 0x11d0, 0xbd, 0x3b, 0x00, 0xa0, 0xc9, 0x11, 0xce, 0x86);


//////列出USB摄像头设备列表
int ListDevices(vector<string>& list)
{
	ICreateDevEnum *pDevEnum = NULL;
	IEnumMoniker *pEnum = NULL;
	int deviceCounter = 0;
	CoInitialize(NULL);

	HRESULT hr = CoCreateInstance(
		CLSID_SystemDeviceEnum,
		NULL,
		CLSCTX_INPROC_SERVER,
		IID_ICreateDevEnum,
		reinterpret_cast<void**>(&pDevEnum)
		);

	if (SUCCEEDED(hr))
	{
		hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
		if (hr == S_OK) {

			IMoniker *pMoniker = NULL;
			while (pEnum->Next(1, &pMoniker, NULL) == S_OK)
			{
				IPropertyBag *pPropBag;
				hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag,
					(void**)(&pPropBag));

				if (FAILED(hr)) {
					pMoniker->Release();
					continue;        // Skip this one, maybe the next one will work.
				}

				VARIANT varName;
				VariantInit(&varName);
				hr = pPropBag->Read(L"Description", &varName, 0);
				if (FAILED(hr))
				{
					hr = pPropBag->Read(L"FriendlyName", &varName, 0);
				}

				if (SUCCEEDED(hr))
				{
					hr = pPropBag->Read(L"FriendlyName", &varName, 0);
					int count = 0;
					char tmp[255] = { 0 };
					while (varName.bstrVal[count] != 0x00 && count < 255)
					{
						tmp[count] = (char)varName.bstrVal[count];
						count++;
					}
					list.push_back(tmp);
				}

				pPropBag->Release();
				pPropBag = NULL;
				pMoniker->Release();
				pMoniker = NULL;

				deviceCounter++;
			}

			pDevEnum->Release();
			pDevEnum = NULL;
			pEnum->Release();
			pEnum = NULL;
		}
	}
	return deviceCounter;
}


CaptureThread::CaptureThread(QObject *parent, int device, std::string str) :
	QThread(parent)
{
	pause_status = true;
	quitFlag = false;
	devicenum = device;

	width = 6000;
	height = 4000;
	/////最大的2400W像素
	InputArray = (unsigned char *)malloc(width * height * 3);
	CameraDeviceStr = str;
	FrameCount = 0;
}


void CaptureThread::release()
{
	free(InputArray);
	free(OutputImg);
}


void CaptureThread::run()
{
	forever
	{
		if (!pause_status)
		{
			if (quitFlag)
				break;
			if (CameraGetImageBuffer(devicenum, &g_tFrameHead, &g_pRawBuffer, 200) == CAMERA_STATUS_SUCCESS)
			{
				if (quitFlag)
					break;

				CameraImageProcess(devicenum, g_pRawBuffer, (unsigned char *)InputArray, &g_tFrameHead);

				if (FrameCount == 0)
				{
					OutputImg = new cv::Mat(g_tFrameHead.iHeight, g_tFrameHead.iWidth, CV_8UC1);
				}
				CameraReleaseImageBuffer(devicenum,g_pRawBuffer);
				OutputImg->data = (unsigned char *)InputArray;
				FrameCount++;
				emit captured(OutputImg);
			}
			else 
			{
				printf("timeout \n");
				usleep(1000);
			}
		}
		else usleep(1000);
		if (quitFlag) break;
	}
}


void CaptureThread::stream()
{
	pause_status = false;
}


void CaptureThread::pause()
{
	pause_status = true;
}


void CaptureThread::stop()
{
	pause_status = true;
	quitFlag = true;
}