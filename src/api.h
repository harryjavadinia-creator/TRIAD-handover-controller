#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define HandoverInterceptionController_DLLIMPORT __declspec(dllimport)
#  define HandoverInterceptionController_DLLEXPORT __declspec(dllexport)
#  define HandoverInterceptionController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define HandoverInterceptionController_DLLIMPORT __attribute__((visibility("default")))
#    define HandoverInterceptionController_DLLEXPORT __attribute__((visibility("default")))
#    define HandoverInterceptionController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define HandoverInterceptionController_DLLIMPORT
#    define HandoverInterceptionController_DLLEXPORT
#    define HandoverInterceptionController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef HandoverInterceptionController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HandoverInterceptionController_DLLAPI
#  define HandoverInterceptionController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef HandoverInterceptionController_EXPORTS
#    define HandoverInterceptionController_DLLAPI HandoverInterceptionController_DLLEXPORT
#  else
#    define HandoverInterceptionController_DLLAPI HandoverInterceptionController_DLLIMPORT
#  endif // HandoverInterceptionController_EXPORTS
#  define HandoverInterceptionController_LOCAL HandoverInterceptionController_DLLLOCAL
#endif // HandoverInterceptionController_STATIC