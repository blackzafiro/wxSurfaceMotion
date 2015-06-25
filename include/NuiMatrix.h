#pragma once

#include "windows.h"
#include <NuiApi.h>

namespace NuiMatrix {
	/// <summary>
	/// Set Identity in a Matrix4
	/// </summary>
	/// <param name="mat">The matrix to set to identity</param>
	void SetIdentityMatrix(Matrix4 &mat);
}