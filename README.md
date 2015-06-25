wxSurfaceMotion
=================

Capture of object shape deformation with a Kinect3D

Tested with target platform x64

Dependencies:
-------------

* Compiled with Microsoft Visual Studio Express 2012 for Windows Desktop
* Thread management with Boost.Thread from Boost 1.57.0
* GUI with wxWdigets 3.0.2
* Kinect Developer Toolkit 1.8.0 and KinectSDK v1.8 from Microsoft

To compile wxSurfaceMotion, Boost and wxWidgets source code is required.
* Let %WXWIDGETS_HOME% be the directory where wxWdigets was built.
* Let %BOOST_HOME% be the directory where Boost was built.

Verify that the following settings are correct in the Property Pages before compiling wxSurfaceMotion.


# wxWidgets

Download from:

https://www.wxwidgets.org/

Build wxWidgets as Debug|x64, Configuration Properties->General->Configuration type->Static library (.lib):

For wxSurfaceMotion:
* Configuration Properties->General->Character Set->Use Unicode Character Set
* Configuration Properties->C/C++ ->General->SDL Checks->No (/sdl-)
* Configuration Properties->C/C++ ->Code Generation->Runtime Library->Multi-threaded Debug (/MTd)

Add to include files:

> Configuration Properties->C/C++ ->General->Additional Include Directories

* %WXWIDGETS_HOME%\include
* %WXWIDGETS_HOME%\include\msvc

Add to additional library directories:

> Configuration Properties->Linker->General->Additional Library Directories

* %WXWIDGETS_HOME%\lib\vc_x64_lib



# Boost.Thread

Download from:

http://www.boost.org/

Compile with:
> bootstrap
> .\b2 toolset=msvc-11.0 threading=multi address-model=64 runtime-link=static

For wxSurfaceMotion:

Add to include files:

> Configuration Properties->C/C++ ->General->Additional Include Directories

* %BOOST_HOME%

Add to additional library directories:

> Configuration Properties->Linker->General->Additional Library Directories

* %BOOST_HOME%\stage\lib



# Kinect Development Toolkit

Dowload from:

[Kinect for Windows SDK v1.8 and Kinect Development Toolkit v1.8](http://www.microsoft.com/en-us/download/details.aspx?id=40276)

For wxSurfaceMotion:

Add to include files:

> Configuration Properties->C/C++ ->General->Additional Include Directories

* C:\Program Files\Microsoft SDKs\Kinect\Developer Toolkit v1.8.0\inc
* C:\Program Files\Microsoft SDKs\Kinect\v1.8\inc

Add to additional library directories:

> Configuration Properties->Linker->General->Additional Library Directories

* C:\Program Files\Microsoft SDKs\Kinect\Developer Toolkit v1.8.0\Lib\amd64
* C:\Program Files\Microsoft SDKs\Kinect\v1.8\lib\amd64

> Configuration Properties->Linker->Input->Additional Dependencies

Additional Dependencies: Kinect10.lib
