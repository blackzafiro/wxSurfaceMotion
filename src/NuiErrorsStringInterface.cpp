#include "NuiErrorsStringInterface.h"

namespace NuiErrorsStringInterface {
	std::string NuiFusionDepthToDepthFloatFrameError(HRESULT hr){
		switch(hr) {
		case E_INVALIDARG:
			return std::string("E_INVALIDARG");
			break;
		case E_NOINTERFACE:
			return std::string("E_NOINTERFACE");
			break;
		case E_OUTOFMEMORY:
			return std::string("E_OUTOFMEMORY");
			break;
		/*case E_GPU_OUTOFMEMORY:
			return std::string("E_GPU_OUTOFMEMORY");
			break;*/
		case E_NUI_FEATURE_NOT_INITIALIZED:
			return std::string("E_NUI_FEATURE_NOT_INITIALIZED");
			break;
		case E_NUI_DEVICE_NOT_CONNECTED:
			return std::string("E_NUI_DEVICE_NOT_CONNECTED");
			break;
		case E_FAIL:
			return std::string("E_FAIL - unknown reason.");
			break;
		}
	}
}