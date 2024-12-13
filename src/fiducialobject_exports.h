
#ifndef FIDUCIALOBJECT_EXPORTS_H
#define FIDUCIALOBJECT_EXPORTS_H

 #if !defined _CRT_SECURE_NO_DEPRECATE && _MSC_VER > 1300
#define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio 2005 warnings */
#endif

#if (defined WIN32 || defined _WIN32 || defined WINCE)   && defined FIDUCIALOBJECT_DSO_EXPORTS
#define FIDUCIALOBJECT_API __declspec(dllexport)
#pragma warning ( disable : 4251 ) //disable warning to templates with dll linkage.
#pragma warning ( disable : 4290 ) //disable warning due to exception specifications.
#pragma warning ( disable : 4996 ) //disable warning regarding unsafe vsprintf.
#pragma warning ( disable : 4244 ) //disable warning convesions with lost of data.

#else
#define FIDUCIALOBJECT_API
#endif


#define  FIDUCIALOBJECT_VERSION "1.0.0"
#endif // FIDUCIALOBJECT_EXPORTS_H
