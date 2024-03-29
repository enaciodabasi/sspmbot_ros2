#ifndef MECANUM_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
#define MECANUM_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#    ifdef __GNUC__
#       define MECANUM_DRIVE_CONTROLLER_EXPORT __attribute__((dllexport))
#       define MECANUM_DRIVE_CONTROLLER_IMPORT __attribute__((dllimport))
#   else
#       define MECANUM_DRIVE_CONTROLLER_EXPORT __declspec(dllexport)
#       define MECANUM_DRIVE_CONTROLLER_IMPORT __declspec(dllimport)
#   endif
#   ifdef MECANUM_DRIVE_CONTROLLER_BUILDING_DLL
#       define MECANUM_DRIVE_CONTROLLER_PUBLIC MECANUM_DRIVE_CONTROLLER_EXPORT
#   else
#       define MECANUM_DRIVE_CONTROLLER_PUBLIC MECANUM_DRIVE_CONTROLLER_IMPORT
#   endif
#   define MECANUM_DRIVE_CONTROLLER_PUBLIC_TYPE MECANUM_DRIVE_CONTROLLER_PUBLIC
#   define MECANUM_DRIVE_CONTROLLER_LOCAL
#else
#   define MECANUM_DRIVE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#   define MECANUM_DRIVE_CONTROLLER_IMPORT
#   if __GNUC__ >= 4
#       define MECANUM_DRIVE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#       define MECANUM_DRIVE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#   else
#       define MECANUM_DRIVE_CONTROLLER_PUBLIC
#       define MECANUM_DRIVE_CONTROLLER_LOCAL
#   endif
#   define MECANUM_DRIVE_CONTROLLER_PUBLIC_TYPE
#endif

#endif