#include "NuiMatrix.h"

namespace NuiMatrix {
	/// <summary>
	/// Set Identity in a Matrix4
	/// </summary>
	/// <param name="mat">The matrix to set to identity</param>
	void SetIdentityMatrix(Matrix4 &mat)
	{
		mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
		mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
		mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
		mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
	}
}