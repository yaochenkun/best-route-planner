# Microsoft Developer Studio Project File - Name="物流配送最优路径规划模拟系统" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=物流配送最优路径规划模拟系统 - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "物流配送最优路径规划模拟系统.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "物流配送最优路径规划模拟系统.mak" CFG="物流配送最优路径规划模拟系统 - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "物流配送最优路径规划模拟系统 - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE "物流配送最优路径规划模拟系统 - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "物流配送最优路径规划模拟系统 - Win32 Release"

# PROP BASE Use_MFC 6
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 6
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /Yu"stdafx.h" /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Yu"stdafx.h" /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x804 /d "NDEBUG" /d "_AFXDLL"
# ADD RSC /l 0x804 /d "NDEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /subsystem:windows /machine:I386
# ADD LINK32 /nologo /subsystem:windows /machine:I386

!ELSEIF  "$(CFG)" == "物流配送最优路径规划模拟系统 - Win32 Debug"

# PROP BASE Use_MFC 6
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 6
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /Yu"stdafx.h" /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Yu"stdafx.h" /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x804 /d "_DEBUG" /d "_AFXDLL"
# ADD RSC /l 0x804 /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 Msimg32.lib libmysql.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept

!ENDIF 

# Begin Target

# Name "物流配送最优路径规划模拟系统 - Win32 Release"
# Name "物流配送最优路径规划模拟系统 - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\CarDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\HelpDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\HelpDlg2.cpp
# End Source File
# Begin Source File

SOURCE=.\LawDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\QueryDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\StdAfx.cpp
# ADD CPP /Yc"stdafx.h"
# End Source File
# Begin Source File

SOURCE=".\物流配送最优路径规划模拟系统.cpp"
# End Source File
# Begin Source File

SOURCE=".\物流配送最优路径规划模拟系统.rc"
# End Source File
# Begin Source File

SOURCE=".\物流配送最优路径规划模拟系统Dlg.cpp"
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\CarDlg.h
# End Source File
# Begin Source File

SOURCE=.\HelpDlg.h
# End Source File
# Begin Source File

SOURCE=.\HelpDlg2.h
# End Source File
# Begin Source File

SOURCE=.\LawDlg.h
# End Source File
# Begin Source File

SOURCE=.\QueryDlg.h
# End Source File
# Begin Source File

SOURCE=.\Resource.h
# End Source File
# Begin Source File

SOURCE=.\StdAfx.h
# End Source File
# Begin Source File

SOURCE=".\物流配送最优路径规划模拟系统.h"
# End Source File
# Begin Source File

SOURCE=".\物流配送最优路径规划模拟系统Dlg.h"
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# Begin Source File

SOURCE=.\res\10000810692626.bmp
# End Source File
# Begin Source File

SOURCE=.\res\103.bmp
# End Source File
# Begin Source File

SOURCE=.\res\108.bmp
# End Source File
# Begin Source File

SOURCE=.\res\5.ico
# End Source File
# Begin Source File

SOURCE=.\res\93.bmp
# End Source File
# Begin Source File

SOURCE=.\res\AimMode.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Arrive.bmp
# End Source File
# Begin Source File

SOURCE=.\res\BeenClient.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Beginning.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Client.bmp
# End Source File
# Begin Source File

SOURCE=.\res\CoverMap.bmp
# End Source File
# Begin Source File

SOURCE=.\res\DoubleDown.bmp
# End Source File
# Begin Source File

SOURCE=.\res\flag.bmp
# End Source File
# Begin Source File

SOURCE=".\res\map - 副本.bmp"
# End Source File
# Begin Source File

SOURCE=.\res\map.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Menu.bmp
# End Source File
# Begin Source File

SOURCE=.\res\OnMap.bmp
# End Source File
# Begin Source File

SOURCE=.\res\RandMode.bmp
# End Source File
# Begin Source File

SOURCE=.\res\RightMenu.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Road.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Running.bmp
# End Source File
# Begin Source File

SOURCE=.\res\star.bmp
# End Source File
# Begin Source File

SOURCE=".\res\Store and Client.bmp"
# End Source File
# Begin Source File

SOURCE=.\res\Store.bmp
# End Source File
# Begin Source File

SOURCE=.\res\StoreFlag.bmp
# End Source File
# Begin Source File

SOURCE=.\res\Traffic.bmp
# End Source File
# Begin Source File

SOURCE=".\res\仓库旗.bmp"
# End Source File
# Begin Source File

SOURCE=".\res\客户旗.bmp"
# End Source File
# Begin Source File

SOURCE=".\res\物流配送最优路径规划模拟系统.ico"
# End Source File
# Begin Source File

SOURCE=".\res\物流配送最优路径规划模拟系统.rc2"
# End Source File
# End Group
# Begin Source File

SOURCE=.\ReadMe.txt
# End Source File
# End Target
# End Project
