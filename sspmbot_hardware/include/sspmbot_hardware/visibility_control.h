#ifndef SSPMBOT_HARDWARE__VISIBILITY_CONTROL_H
#define SSPMBOT_HARDWARE__VISIBILITY_CONTROL_H

#if defined _WIN32 || defined __CYGWIN__
#    ifdef __GNUC__
#       define SSPMBOT_HARDWARE_EXPORT __attribute__((dllexport))
#       define SSPMBOT_HARDWARE_IMPORT __attribute__((dllimport))
#   else
#       define SSPMBOT_HARDWARE_EXPORT __declspec(dllexport)
#       define SSPMBOT_HARDWARE_IMPORT __declspec(dllimport)
#   endif
#   ifdef SSPMBOT_HARDWARE_BUILDING_DLL
#       define SSPMBOT_HARDWARE_PUBLIC SSPMBOT_HARDWARE_EXPORT
#   else
#       define SSPMBOT_HARDWARE_PUBLIC SSPMBOT_HARDWARE_IMPORT
#   endif
#   define SSPMBOT_HARDWARE_PUBLIC_TYPE SSPMBOT_HARDWARE_PUBLIC
#   define SSPMBOT_HARDWARE_LOCAL
#else
#   define SSPMBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#   define SSPMBOT_HARDWARE_IMPORT
#   if __GNUC__ >= 4
#       define SSPMBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#       define SSPMBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#   else
#       define SSPMBOT_HARDWARE_PUBLIC
#       define SSPMBOT_HARDWARE_LOCAL
#   endif
#   define SSPMBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif // SSPMBOT_HARDWARE__VISIBILITY_CONTROL_H