/*
 *  duk_config.h configuration header
 *
 *  Git commit: e483716f032a2715c0b455d82779af7bdc914f5f
 *  Git describe: v2.5.0-354-ge483716f
 *  Git branch: master
 *
 *  Supported platforms:
 *      - macOS, iOS, watchOS, tvOS, Darwin, etc
 *      - Orbis
 *      - MS-DOS
 *      - OpenBSD
 *      - Generic BSD
 *      - Atari ST TOS
 *      - AmigaOS
 *      - Durango (XboxOne)
 *      - Windows
 *      - Flashplayer (Crossbridge)
 *      - QNX
 *      - TI-Nspire
 *      - Emscripten
 *      - Android
 *      - Linux
 *      - Solaris
 *      - AIX
 *      - HPUX
 *      - Generic POSIX
 *      - Cygwin
 *      - Generic UNIX
 *      - Generic fallback
 *
 *  Supported architectures:
 *      - x86
 *      - x64
 *      - x32
 *      - ARM 32-bit
 *      - ARM 64-bit
 *      - MIPS 32-bit
 *      - MIPS 64-bit
 *      - PowerPC 32-bit
 *      - PowerPC 64-bit
 *      - SPARC 32-bit
 *      - SPARC 64-bit
 *      - RISC-V 32-bit
 *      - RISC-V 64-bit
 *      - SuperH
 *      - Motorola 68k
 *      - Emscripten
 *      - Generic
 *
 *  Supported compilers:
 *      - Clang
 *      - GCC
 *      - MSVC
 *      - Emscripten
 *      - TinyC
 *      - VBCC
 *      - Bruce's C compiler
 *      - Generic
 *
 */

#if !defined(DUK_CONFIG_H_INCLUDED)
#define DUK_CONFIG_H_INCLUDED

/* Recommended config option DUK_USE_FATAL_HANDLER not provided */

/*
 *  Intermediate helper defines
 */

/* Not configured for DLL build. */
#undef DUK_F_DLL_BUILD

/* AIX */
#if defined(_AIX)
/* defined(__xlc__) || defined(__IBMC__): works but too wide */
#define DUK_F_AIX
#endif

/* AmigaOS.  Neither AMIGA nor __amigaos__ is defined on VBCC, so user must
 * define 'AMIGA' manually when using VBCC.
 */
#if defined(AMIGA) || defined(__amigaos__)
#define DUK_F_AMIGAOS
#endif

#if defined(ANDROID) || defined(__ANDROID__)
#define DUK_F_ANDROID
#endif

/* Apple macOS, iOS, watchOS, tvOS, Darwin, etc */
#if defined(__APPLE__)
#define DUK_F_APPLE
#endif

/* ARM */
#if defined(__arm__) || defined(__thumb__) || defined(_ARM) || defined(_M_ARM) || defined(_M_ARM64) || defined(__aarch64__)
#define DUK_F_ARM
#if defined(__LP64__) || defined(_LP64) || defined(__arm64) || defined(__arm64__) || defined(_M_ARM64) || defined(__aarch64__)
#define DUK_F_ARM64
#else
#define DUK_F_ARM32
#endif
#endif

/* BCC (Bruce's C compiler): this is a "torture target" for compilation */
#if defined(__BCC__) || defined(__BCC_VERSION__)
#define DUK_F_BCC
#endif

/* BSD variant */
#if defined(DUK_F_FREEBSD) || defined(DUK_F_NETBSD) || defined(DUK_F_OPENBSD) || \
    defined(__bsdi__) || defined(__DragonFly__)
#define DUK_F_BSD
#endif

/* C99 or above */
#undef DUK_F_C99
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
#define DUK_F_C99
#endif

/* Clang */
#if defined(__clang__)
#define DUK_F_CLANG
#endif

/* C++ */
#undef DUK_F_CPP
#if defined(__cplusplus)
#define DUK_F_CPP
#endif

/* C++11 or above */
#undef DUK_F_CPP11
#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define DUK_F_CPP11
#endif

/* Cygwin */
#if defined(__CYGWIN__)
#define DUK_F_CYGWIN
#endif

/* DJGPP (MSDOS), see https://sourceforge.net/p/predef/wiki/Compilers/ */
#if defined(__DJGPP__)
#define DUK_F_DJGPP
#define DUK_F_DJGPP_MAJOR __DJGPP__
#if defined(__DJGPP_MINOR__)
#define DUK_F_DJGPP_MINOR __DJGPP_MINOR__
#else
#define DUK_F_DJGPP_MINOR 0
#endif
#define DUK_F_DJGPP_PATCH 0
#define DUK_F_DJGPP_VERSION (DUK_F_DJGPP_MAJOR * 10000L + DUK_F_DJGPP_MINOR * 100L + DUK_F_DJGPP_PATCH)
#endif

/* Durango (Xbox One) */
#if defined(_DURANGO) || defined(_XBOX_ONE)
#define DUK_F_DURANGO
#endif

/* Emscripten (provided explicitly by user), improve if possible */
#if defined(EMSCRIPTEN)
#define DUK_F_EMSCRIPTEN
#endif

/* Flash player (e.g. Crossbridge) */
#if defined(__FLASHPLAYER__)
#define DUK_F_FLASHPLAYER
#endif

/* FreeBSD */
#if defined(__FreeBSD__) || defined(__FreeBSD)
#define DUK_F_FREEBSD
#endif

/* GCC.  Clang also defines __GNUC__ so don't detect GCC if using Clang. */
#if defined(__GNUC__) && !defined(__clang__) && !defined(DUK_F_CLANG)
#define DUK_F_GCC
#if defined(__GNUC__) && defined(__GNUC_MINOR__) && defined(__GNUC_PATCHLEVEL__)
/* Convenience, e.g. gcc 4.5.1 == 40501; http://stackoverflow.com/questions/6031819/emulating-gccs-builtin-unreachable */
#define DUK_F_GCC_VERSION  (__GNUC__ * 10000L + __GNUC_MINOR__ * 100L + __GNUC_PATCHLEVEL__)
#else
#error cannot figure out gcc version
#endif
#endif

/* HPUX */
#if defined(__hpux)
#define DUK_F_HPUX
#if defined(__ia64)
#define DUK_F_HPUX_ITANIUM
#endif
#endif

/* Linux */
#if defined(__linux) || defined(__linux__) || defined(linux)
#define DUK_F_LINUX
#endif

/* Motorola 68K.  Not defined by VBCC, so user must define one of these
 * manually when using VBCC.
 */
#if defined(__m68k__) || defined(M68000) || defined(__MC68K__)
#define DUK_F_M68K
#endif

/* MinGW.  Also GCC flags (DUK_F_GCC) are enabled now. */
#if defined(__MINGW32__) || defined(__MINGW64__)
#define DUK_F_MINGW
#endif

/* Atari Mint */
#if defined(__MINT__)
#define DUK_F_MINT
#endif

/* MIPS.  Related defines: __MIPSEB__, __MIPSEL__, __mips_isa_rev, __LP64__ */
#if defined(__mips__) || defined(mips) || defined(_MIPS_ISA) || \
    defined(_R3000) || defined(_R4000) || defined(_R5900) || \
    defined(_MIPS_ISA_MIPS1) || defined(_MIPS_ISA_MIPS2) || \
    defined(_MIPS_ISA_MIPS3) || defined(_MIPS_ISA_MIPS4) || \
    defined(__mips) || defined(__MIPS__)
#define DUK_F_MIPS
#if defined(__LP64__) || defined(_LP64) || defined(__mips64) || \
    defined(__mips64__) || defined(__mips_n64)
#define DUK_F_MIPS64
#else
#define DUK_F_MIPS32
#endif
#endif

#if defined(__MSDOS__) || defined(__MSDOS) || defined(MSDOS)
#define DUK_F_MSDOS
#endif

/* MSVC */
#if defined(_MSC_VER)
/* MSVC preprocessor defines: http://msdn.microsoft.com/en-us/library/b0084kay.aspx
 * _MSC_FULL_VER includes the build number, but it has at least two formats, see e.g.
 * BOOST_MSVC_FULL_VER in http://www.boost.org/doc/libs/1_52_0/boost/config/compiler/visualc.hpp
 */
#define DUK_F_MSVC
#if defined(_MSC_FULL_VER)
#if (_MSC_FULL_VER > 100000000)
#define DUK_F_MSVC_FULL_VER _MSC_FULL_VER
#else
#define DUK_F_MSCV_FULL_VER (_MSC_FULL_VER * 10)
#endif
#endif
#endif  /* _MSC_VER */

/* NetBSD */
#if defined(__NetBSD__) || defined(__NetBSD)
#define DUK_F_NETBSD
#endif

/* illumos / Solaris */
#if defined(__sun) && defined(__SVR4)
#define DUK_F_SUN
#if defined(__SUNPRO_C) && (__SUNPRO_C < 0x550)
#define DUK_F_OLD_SOLARIS
/* Defines _ILP32 / _LP64 required by DUK_F_X86/DUK_F_X64.  Platforms
 * are processed before architectures, so this happens before the
 * DUK_F_X86/DUK_F_X64 detection is emitted.
 */
#include <sys/isa_defs.h>
#endif
#endif

/* OpenBSD */
#if defined(__OpenBSD__) || defined(__OpenBSD)
#define DUK_F_OPENBSD
#endif

/* Orbis (PS4) variant */
#if defined(DUK_F_FREEBSD) && defined(__ORBIS__)
#define DUK_F_ORBIS
#endif

/* POSIX */
#if defined(__posix)
#define DUK_F_POSIX
#endif

/* PowerPC */
#if defined(__powerpc) || defined(__powerpc__) || defined(__PPC__)
#define DUK_F_PPC
#if defined(__PPC64__) || defined(__LP64__) || defined(_LP64)
#define DUK_F_PPC64
#else
#define DUK_F_PPC32
#endif
#endif

/* QNX */
#if defined(__QNX__)
#define DUK_F_QNX
#endif

/* RISC-V, https://github.com/riscv/riscv-toolchain-conventions#cc-preprocessor-definitions */
#if defined(__riscv)
#define DUK_F_RISCV
#if defined(__riscv_xlen)
#if (__riscv_xlen == 32)
#define DUK_F_RISCV32
#elif (__riscv_xlen == 64)
#define DUK_F_RISCV64
#else
#error __riscv_xlen has unsupported value (not 32 or 64)
#endif
#else
#error __riscv defined without __riscv_xlen
#endif
#endif  /* __riscv */

/* SPARC */
#if defined(sparc) || defined(__sparc) || defined(__sparc__)
#define DUK_F_SPARC
#if defined(__LP64__) || defined(_LP64)
#define DUK_F_SPARC64
#else
#define DUK_F_SPARC32
#endif
#endif

/* SuperH */
#if defined(__sh__) || \
    defined(__sh1__) || defined(__SH1__) || \
    defined(__sh2__) || defined(__SH2__) || \
    defined(__sh3__) || defined(__SH3__) || \
    defined(__sh4__) || defined(__SH4__) || \
    defined(__sh5__) || defined(__SH5__)
#define DUK_F_SUPERH
#endif

/* TI-Nspire (using Ndless) */
#if defined(_TINSPIRE)
#define DUK_F_TINSPIRE
#endif

/* TinyC */
#if defined(__TINYC__)
/* http://bellard.org/tcc/tcc-doc.html#SEC9 */
#define DUK_F_TINYC
#endif

/* Atari ST TOS.  __TOS__ defined by PureC.  No platform define in VBCC
 * apparently, so to use with VBCC user must define __TOS__ manually.
  */
#if defined(__TOS__)
#define DUK_F_TOS
#endif

/* Generic Unix (includes Cygwin) */
#if defined(__unix) || defined(__unix__) || defined(unix) || \
    defined(DUK_F_LINUX) || defined(DUK_F_BSD)
#define DUK_F_UNIX
#endif

/* VBCC */
#if defined(__VBCC__)
#define DUK_F_VBCC
#endif

/* Windows, both 32-bit and 64-bit */
#if defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
    defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__)
#define DUK_F_WINDOWS
#if defined(_WIN64) || defined(WIN64)
#define DUK_F_WIN64
#else
#define DUK_F_WIN32
#endif
#endif

/* Intel x86 (32-bit), x64 (64-bit) or x32 (64-bit but 32-bit pointers),
 * define only one of DUK_F_X86, DUK_F_X64, DUK_F_X32.
 * https://sites.google.com/site/x32abi/
 *
 * With DUK_F_OLD_SOLARIS the <sys/isa_defs.h> header must be included
 * before this.
 */
#if defined(__amd64__) || defined(__amd64) || \
    defined(__x86_64__) || defined(__x86_64) || \
    defined(_M_X64) || defined(_M_AMD64)
#if defined(__ILP32__) || defined(_ILP32)
#define DUK_F_X32
#else
#define DUK_F_X64
#endif
#elif defined(i386) || defined(__i386) || defined(__i386__) || \
      defined(__i486__) || defined(__i586__) || defined(__i686__) || \
      defined(__IA32__) || defined(_M_IX86) || defined(__X86__) || \
      defined(_X86_) || defined(__THW_INTEL__) || defined(__I86__)
#if defined(__LP64__) || defined(_LP64)
/* This should not really happen, but would indicate x64. */
#define DUK_F_X64
#else
#define DUK_F_X86
#endif
#endif

/*
 *  Platform autodetection
 */

/* Workaround for older C++ compilers before including <inttypes.h>,
 * see e.g.: https://sourceware.org/bugzilla/show_bug.cgi?id=15366
 */
#if defined(__cplusplus) && !defined(__STDC_LIMIT_MACROS)
#define __STDC_LIMIT_MACROS
#endif
#if defined(__cplusplus) && !defined(__STDC_CONSTANT_MACROS)
#define __STDC_CONSTANT_MACROS
#endif

#if defined(DUK_F_APPLE)
/* --- macOS, iOS, watchOS, tvOS, Darwin, etc --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <TargetConditionals.h>
#include <architecture/byte_order.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

/* http://stackoverflow.com/questions/5919996/how-to-detect-reliably-mac-os-x-ios-linux-windows-in-c-preprocessor */
/* https://stackoverflow.com/questions/12132933/preprocessor-macro-for-os-x-targets */
/* Some of the combinations below probably don't actually exist
 * (like macOS simulator) but include them for completeness.
 */
#if TARGET_OS_SIMULATOR && TARGET_OS_MACCATALYST
#define DUK_USE_OS_STRING "catalyst-sim"
#elif TARGET_OS_SIMULATOR && TARGET_OS_BRIDGE
#define DUK_USE_OS_STRING "bridgeos-sim"
#elif TARGET_OS_SIMULATOR && TARGET_OS_WATCH
#define DUK_USE_OS_STRING "watchos-sim"
#elif TARGET_OS_SIMULATOR && TARGET_OS_TV
#define DUK_USE_OS_STRING "tvos-sim"
#elif TARGET_OS_SIMULATOR && TARGET_OS_IOS
#define DUK_USE_OS_STRING "ios-sim"
#elif TARGET_IPHONE_SIMULATOR && TARGET_OS_IOS  /* deprecated */
#define DUK_USE_OS_STRING "ios-sim"
#elif TARGET_OS_SIMULATOR && TARGET_OS_MAC
#define DUK_USE_OS_STRING "macos-sim"
#elif TARGET_OS_MACCATALYST
#define DUK_USE_OS_STRING "catalyst"
#elif TARGET_OS_BRIDGE
#define DUK_USE_OS_STRING "bridgeos"
#elif TARGET_OS_WATCH
#define DUK_USE_OS_STRING "watchos"
#elif TARGET_OS_TV
#define DUK_USE_OS_STRING "tvos"
#elif TARGET_OS_IOS
#define DUK_USE_OS_STRING "ios"
#elif TARGET_OS_MAC
#define DUK_USE_OS_STRING "macos"
#else
#define DUK_USE_OS_STRING "osx-unknown"
#endif

/* Use _setjmp() on Apple by default, see GH-55. */
#define DUK_JMPBUF_TYPE       jmp_buf
#define DUK_SETJMP(jb)        _setjmp((jb))
#define DUK_LONGJMP(jb)       _longjmp((jb), 1)
#elif defined(DUK_F_ORBIS)
/* --- Orbis --- */
/* Orbis = PS4 */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_S
/* no parsing (not an error) */
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <machine/endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING  "orbis"
#elif defined(DUK_F_MSDOS)
/* --- MS-DOS --- */
#if defined(DUK_F_DJGPP)
/* These are for DJGPP, add #ifdefs for other MS-DOS compilers as needed. */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#undef DUK_USE_DATE_PRS_STRPTIME  /* Not supported, see: https://github.com/svaarala/duktape/issues/2472 */
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>
#include <sys/time.h>
#else
/* Placeholder, unlikely to work outside of DJGPP. */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#undef DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>
#include <sys/time.h>
#endif

#define DUK_USE_OS_STRING "msdos"
#elif defined(DUK_F_OPENBSD)
/* --- OpenBSD --- */
/* http://www.monkey.org/openbsd/archive/ports/0401/msg00089.html */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <sys/endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING  "openbsd"
#elif defined(DUK_F_BSD)
/* --- Generic BSD --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <sys/endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING  "bsd"
#elif defined(DUK_F_TOS)
/* --- Atari ST TOS --- */
#define DUK_USE_DATE_NOW_TIME
#define DUK_USE_DATE_TZO_GMTIME
/* no parsing (not an error) */
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>

#define DUK_USE_OS_STRING  "tos"

/* TOS on M68K is always big endian. */
#if !defined(DUK_USE_BYTEORDER) && defined(DUK_F_M68K)
#define DUK_USE_BYTEORDER 3
#endif
#elif defined(DUK_F_AMIGAOS)
/* --- AmigaOS --- */
#if defined(DUK_F_M68K)
/* AmigaOS on M68k */
#define DUK_USE_DATE_NOW_TIME
#define DUK_USE_DATE_TZO_GMTIME
/* no parsing (not an error) */
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>
#elif defined(DUK_F_PPC)
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>
#if !defined(UINTPTR_MAX)
#define UINTPTR_MAX UINT_MAX
#endif
#else
#error AmigaOS but not M68K/PPC, not supported now
#endif

#define DUK_USE_OS_STRING "amigaos"

/* AmigaOS on M68K or PPC is always big endian. */
#if !defined(DUK_USE_BYTEORDER) && (defined(DUK_F_M68K) || defined(DUK_F_PPC))
#define DUK_USE_BYTEORDER 3
#endif
#elif defined(DUK_F_DURANGO)
/* --- Durango (XboxOne) --- */
/* Durango = XboxOne
 * Configuration is nearly identical to Windows, except for
 * DUK_USE_DATE_TZO_WINDOWS.
 */

/* Initial fix: disable secure CRT related warnings when compiling Duktape
 * itself (must be defined before including Windows headers).  Don't define
 * for user code including duktape.h.
 */
#if defined(DUK_COMPILING_DUKTAPE) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

/* MSVC does not have sys/param.h */
#define DUK_USE_DATE_NOW_WINDOWS
#define DUK_USE_DATE_TZO_WINDOWS_NO_DST
/* Note: PRS and FMT are intentionally left undefined for now.  This means
 * there is no platform specific date parsing/formatting but there is still
 * the ISO 8601 standard format.
 */
#if defined(DUK_COMPILING_DUKTAPE)
/* Only include when compiling Duktape to avoid polluting application build
 * with a lot of unnecessary defines.
 */
#include <windows.h>
#endif

#define DUK_USE_OS_STRING "durango"

#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#elif defined(DUK_F_WINDOWS)
/* --- Windows --- */
/* Windows version can't obviously be determined at compile time,
 * but _WIN32_WINNT indicates the minimum version targeted:
 * - https://msdn.microsoft.com/en-us/library/6sehtctf.aspx
 */

/* Initial fix: disable secure CRT related warnings when compiling Duktape
 * itself (must be defined before including Windows headers).  Don't define
 * for user code including duktape.h.
 */
#if defined(DUK_COMPILING_DUKTAPE) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

/* Windows 32-bit and 64-bit are currently the same. */
/* MSVC does not have sys/param.h */

#if defined(DUK_COMPILING_DUKTAPE)
/* Only include when compiling Duktape to avoid polluting application build
 * with a lot of unnecessary defines.
 */
#include <windows.h>
#endif

/* GetSystemTimePreciseAsFileTime() available from Windows 8:
 * https://msdn.microsoft.com/en-us/library/windows/desktop/hh706895(v=vs.85).aspx
 */
#if defined(DUK_USE_DATE_NOW_WINDOWS_SUBMS) || defined(DUK_USE_DATE_NOW_WINDOWS)
/* User forced provider. */
#else
#if defined(_WIN32_WINNT) && (_WIN32_WINNT >= 0x0602)
#define DUK_USE_DATE_NOW_WINDOWS_SUBMS
#else
#define DUK_USE_DATE_NOW_WINDOWS
#endif
#endif

#define DUK_USE_DATE_TZO_WINDOWS

/* Note: PRS and FMT are intentionally left undefined for now.  This means
 * there is no platform specific date parsing/formatting but there is still
 * the ISO 8601 standard format.
 */

/* QueryPerformanceCounter() may go backwards in Windows XP, so enable for
 * Vista and later: https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx#qpc_support_in_windows_versions
 */
#if !defined(DUK_USE_GET_MONOTONIC_TIME_WINDOWS_QPC) && \
    defined(_WIN32_WINNT) && (_WIN32_WINNT >= 0x0600)
#define DUK_USE_GET_MONOTONIC_TIME_WINDOWS_QPC
#endif

#define DUK_USE_OS_STRING "windows"

/* On Windows, assume we're little endian.  Even Itanium which has a
 * configurable endianness runs little endian in Windows.
 */
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#elif defined(DUK_F_FLASHPLAYER)
/* --- Flashplayer (Crossbridge) --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "flashplayer"

#if !defined(DUK_USE_BYTEORDER) && defined(DUK_F_FLASHPLAYER)
#define DUK_USE_BYTEORDER 1
#endif
#elif defined(DUK_F_QNX)
/* --- QNX --- */
#if defined(DUK_F_QNX) && defined(DUK_COMPILING_DUKTAPE)
/* See: /opt/qnx650/target/qnx6/usr/include/sys/platform.h */
#define _XOPEN_SOURCE    600
#define _POSIX_C_SOURCE  200112L
#endif

#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "qnx"
#elif defined(DUK_F_TINSPIRE)
/* --- TI-Nspire --- */
#if defined(DUK_COMPILING_DUKTAPE) && !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE    /* e.g. strptime */
#endif

#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "tinspire"
#elif defined(DUK_F_EMSCRIPTEN)
/* --- Emscripten --- */
#if defined(DUK_COMPILING_DUKTAPE)
#if !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE  200809L
#endif
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE      /* e.g. getdate_r */
#endif
#if !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE    /* e.g. strptime */
#endif
#endif  /* DUK_COMPILING_DUKTAPE */

#include <sys/types.h>
#if defined(DUK_F_BCC)
/* no endian.h */
#else
#include <endian.h>
#endif  /* DUK_F_BCC */
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>
#include <stdint.h>

#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME

#define DUK_USE_OS_STRING "emscripten"
#elif defined(DUK_F_ANDROID)
/* --- Android --- */
#if defined(DUK_COMPILING_DUKTAPE)
#if !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE  200809L
#endif
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE      /* e.g. getdate_r */
#endif
#if !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE    /* e.g. strptime */
#endif
#endif  /* DUK_COMPILING_DUKTAPE */

#include <sys/types.h>
#if defined(DUK_F_BCC)
/* no endian.h or stdint.h */
#else
#include <endian.h>
#include <stdint.h>
#endif  /* DUK_F_BCC */
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME

#if 0  /* XXX: safe condition? */
#define DUK_USE_GET_MONOTONIC_TIME_CLOCK_GETTIME
#endif

#define DUK_USE_OS_STRING "android"
#elif defined(DUK_F_LINUX)
/* --- Linux --- */
#if defined(DUK_COMPILING_DUKTAPE)
#if !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE  200809L
#endif
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE      /* e.g. getdate_r */
#endif
#if !defined(_XOPEN_SOURCE)
#define _XOPEN_SOURCE    /* e.g. strptime */
#endif
#endif  /* DUK_COMPILING_DUKTAPE */

#include <sys/types.h>
#if defined(DUK_F_BCC)
/* no endian.h or stdint.h */
#else
#include <endian.h>
#include <stdint.h>
#endif  /* DUK_F_BCC */
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME

#if 0  /* XXX: safe condition? */
#define DUK_USE_GET_MONOTONIC_TIME_CLOCK_GETTIME
#endif

#define DUK_USE_OS_STRING "linux"
#elif defined(DUK_F_SUN)
/* --- Solaris --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME

#include <sys/types.h>
#if defined(DUK_F_OLD_SOLARIS)
/* Old Solaris with no endian.h, stdint.h */
#define DUK_F_NO_STDINT_H
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 3
#endif
#else  /* DUK_F_OLD_SOLARIS */
#include <sys/param.h>
#endif  /* DUK_F_OLD_SOLARIS */

#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "solaris"
#elif defined(DUK_F_AIX)
/* --- AIX --- */
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 3
#endif
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "aix"
#elif defined(DUK_F_HPUX)
/* --- HPUX --- */
#define DUK_F_NO_STDINT_H
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 3
#endif
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "hpux"
#elif defined(DUK_F_POSIX)
/* --- Generic POSIX --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_USE_OS_STRING "posix"
#elif defined(DUK_F_CYGWIN)
/* --- Cygwin --- */
/* don't use strptime() for now */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_FMT_STRFTIME
#include <sys/types.h>
#include <endian.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>

#define DUK_JMPBUF_TYPE       jmp_buf
#define DUK_SETJMP(jb)        _setjmp((jb))
#define DUK_LONGJMP(jb)       _longjmp((jb), 1)

#define DUK_USE_OS_STRING "windows"
#elif defined(DUK_F_UNIX)
/* --- Generic UNIX --- */
#define DUK_USE_DATE_NOW_GETTIMEOFDAY
#define DUK_USE_DATE_TZO_GMTIME_R
#define DUK_USE_DATE_PRS_STRPTIME
#define DUK_USE_DATE_FMT_STRFTIME
#include <time.h>
#include <sys/time.h>
#define DUK_USE_OS_STRING "unknown"
#else
/* --- Generic fallback --- */
/* The most portable current time provider is time(), but it only has a
 * one second resolution.
 */
#define DUK_USE_DATE_NOW_TIME

/* The most portable way to figure out local time offset is gmtime(),
 * but it's not thread safe so use with caution.
 */
#define DUK_USE_DATE_TZO_GMTIME

/* Avoid custom date parsing and formatting for portability. */
#undef DUK_USE_DATE_PRS_STRPTIME
#undef DUK_USE_DATE_FMT_STRFTIME

/* Rely on C89 headers only; time.h must be here. */
#include <time.h>

#define DUK_USE_OS_STRING "unknown"
#endif  /*autodetect platform*/

/* Shared includes: C89 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>  /* varargs */
#include <setjmp.h>
#include <stddef.h>  /* e.g. ptrdiff_t */
#include <math.h>
#include <limits.h>

/* date.h is omitted, and included per platform */

/* Shared includes: stdint.h is C99 */
#if defined(DUK_F_NO_STDINT_H)
/* stdint.h not available */
#else
/* Technically C99 (C++11) but found in many systems.  On some systems
 * __STDC_LIMIT_MACROS and __STDC_CONSTANT_MACROS must be defined before
 * including stdint.h (see above).
 */
#include <stdint.h>
#endif

/* <exception> is only included if needed, based on DUK_USE_xxx flags. */

/*
 *  Architecture autodetection
 */

#if defined(DUK_F_X86)
/* --- x86 --- */
#define DUK_USE_ARCH_STRING "x86"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif

#define DUK_USE_PACKED_TVAL

/* FreeBSD, -m32, and clang prior to 5.0 has union aliasing issues which
 * break duk_tval copying.  Disable packed duk_tval automatically.
 */
#if defined(DUK_F_FREEBSD) && defined(DUK_F_X86) && \
    defined(__clang__) && defined(__clang_major__) && (__clang_major__ < 5)
#undef DUK_USE_PACKED_TVAL
#endif
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_X64)
/* --- x64 --- */
#define DUK_USE_ARCH_STRING "x64"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_X32)
/* --- x32 --- */
#define DUK_USE_ARCH_STRING "x32"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_ARM32)
/* --- ARM 32-bit --- */
#define DUK_USE_ARCH_STRING "arm32"
/* Byte order varies, so rely on autodetect. */
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_ARM64)
/* --- ARM 64-bit --- */
#define DUK_USE_ARCH_STRING "arm64"
/* Byte order varies, so rely on autodetect. */
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_MIPS32)
/* --- MIPS 32-bit --- */
#define DUK_USE_ARCH_STRING "mips32"
/* MIPS byte order varies so rely on autodetection. */
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_MIPS64)
/* --- MIPS 64-bit --- */
#define DUK_USE_ARCH_STRING "mips64"
/* MIPS byte order varies so rely on autodetection. */
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_PPC32)
/* --- PowerPC 32-bit --- */
#define DUK_USE_ARCH_STRING "ppc32"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 3
#endif
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_PPC64)
/* --- PowerPC 64-bit --- */
#define DUK_USE_ARCH_STRING "ppc64"
/* No forced byteorder (both little and big endian are possible). */
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_SPARC32)
/* --- SPARC 32-bit --- */
#define DUK_USE_ARCH_STRING "sparc32"
/* SPARC byte order varies so rely on autodetection. */
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_SPARC64)
/* --- SPARC 64-bit --- */
#define DUK_USE_ARCH_STRING "sparc64"
/* SPARC byte order varies so rely on autodetection. */
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_RISCV32)
/* --- RISC-V 32-bit --- */
#define DUK_USE_ARCH_STRING "riscv32"
#define DUK_USE_BYTEORDER 1
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_RISCV64)
/* --- RISC-V 64-bit --- */
#define DUK_USE_ARCH_STRING "riscv64"
#define DUK_USE_BYTEORDER 1
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_SUPERH)
/* --- SuperH --- */
#define DUK_USE_ARCH_STRING "sh"
/* Byte order varies, rely on autodetection. */
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_M68K)
/* --- Motorola 68k --- */
#define DUK_USE_ARCH_STRING "m68k"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 3
#endif
#define DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#elif defined(DUK_F_EMSCRIPTEN)
/* --- Emscripten --- */
#define DUK_USE_ARCH_STRING "emscripten"
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#undef DUK_USE_PACKED_TVAL
#define DUK_F_PACKED_TVAL_PROVIDED
#else
/* --- Generic --- */
/* These are necessary wild guesses. */
#define DUK_USE_ARCH_STRING "generic"
/* Rely on autodetection for byte order, alignment, and packed tval. */
#endif  /*autodetect architecture*/

/*
 *  Compiler autodetection
 */

#if defined(DUK_F_CLANG)
/* --- Clang --- */
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
/* C99 / C++11 and above: rely on va_copy() which is required. */
#define DUK_VA_COPY(dest,src) va_copy(dest,src)
#else
/* Clang: assume we have __va_copy() in non-C99 mode. */
#define DUK_VA_COPY(dest,src) __va_copy(dest,src)
#endif

#define DUK_NORETURN(decl)  decl __attribute__((noreturn))

#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_unreachable)
#define DUK_UNREACHABLE()  do { __builtin_unreachable(); } while (0)
#endif
#endif

#define DUK_USE_BRANCH_HINTS
#define DUK_LIKELY(x)    __builtin_expect((x), 1)
#define DUK_UNLIKELY(x)  __builtin_expect((x), 0)
#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_unpredictable)
#define DUK_UNPREDICTABLE(x)  __builtin_unpredictable((x))
#endif
#endif

#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_NOINLINE        __attribute__((noinline))
#define DUK_INLINE          inline
#define DUK_ALWAYS_INLINE   inline __attribute__((always_inline))
#endif

/* DUK_HOT */
/* DUK_COLD */

#if defined(DUK_F_DLL_BUILD) && defined(DUK_F_WINDOWS)
/* MSVC dllexport/dllimport: appropriate __declspec depends on whether we're
 * compiling Duktape or the application.
 */
#if defined(DUK_COMPILING_DUKTAPE)
#define DUK_EXTERNAL_DECL  extern __declspec(dllexport)
#define DUK_EXTERNAL       __declspec(dllexport)
#else
#define DUK_EXTERNAL_DECL  extern __declspec(dllimport)
#define DUK_EXTERNAL       should_not_happen
#endif
#if defined(DUK_SINGLE_FILE)
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#else
#define DUK_INTERNAL_DECL  extern
#define DUK_INTERNAL       /*empty*/
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static
#else
#define DUK_EXTERNAL_DECL  __attribute__ ((visibility("default"))) extern
#define DUK_EXTERNAL       __attribute__ ((visibility("default")))
#if defined(DUK_SINGLE_FILE)
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
/* Minimize warnings for unused internal functions with GCC >= 3.1.1 and
 * Clang.  Based on documentation it should suffice to have the attribute
 * in the declaration only, but in practice some warnings are generated unless
 * the attribute is also applied to the definition.
 */
#define DUK_INTERNAL_DECL  static __attribute__ ((unused))
#define DUK_INTERNAL       static __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#endif
#else
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) __attribute__ ((unused)) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden"))) __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden")))
#endif
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static
#endif

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "clang"
#else
#define DUK_USE_COMPILER_STRING "clang"
#endif

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_USE_VARIADIC_MACROS
#endif

#define DUK_USE_UNION_INITIALIZERS

#undef DUK_USE_FLEX_C99
#undef DUK_USE_FLEX_ZEROSIZE
#undef DUK_USE_FLEX_ONESIZE
#if defined(DUK_F_C99)
#define DUK_USE_FLEX_C99
#else
#define DUK_USE_FLEX_ZEROSIZE
#endif

#define DUK_USE_CLANG_PRAGMAS
#define DUK_USE_PACK_CLANG_ATTR

#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_bswap64)
#define DUK_BSWAP64(x) ((duk_uint64_t) __builtin_bswap64((duk_uint64_t) (x)))
#endif
#if __has_builtin(__builtin_bswap32)
#define DUK_BSWAP32(x) ((duk_uint32_t) __builtin_bswap32((duk_uint32_t) (x)))
#endif
#if __has_builtin(__builtin_bswap16)
#define DUK_BSWAP16(x) ((duk_uint16_t) __builtin_bswap16((duk_uint16_t) (x)))
#endif
#endif
#elif defined(DUK_F_GCC)
/* --- GCC --- */
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
/* C99 / C++11 and above: rely on va_copy() which is required. */
#define DUK_VA_COPY(dest,src) va_copy(dest,src)
#else
/* GCC: assume we have __va_copy() in non-C99 mode. */
#define DUK_VA_COPY(dest,src) __va_copy(dest,src)
#endif

#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 20500L) && (DUK_F_GCC_VERSION < 50000L)
/* Since gcc-2.5.
 *
 * Disabled temporarily in GCC 5+ because of an unresolved noreturn-related
 * issue: https://github.com/svaarala/duktape/issues/2155.
 */
#define DUK_NORETURN(decl)  decl __attribute__((noreturn))
#endif

#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 40500L)
/* Since gcc-4.5. */
#define DUK_UNREACHABLE()  do { __builtin_unreachable(); } while (0)
#endif

#define DUK_USE_BRANCH_HINTS
#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 40500L)
/* GCC: test not very accurate; enable only in relatively recent builds
 * because of bugs in gcc-4.4 (http://lists.debian.org/debian-gcc/2010/04/msg00000.html)
 */
#define DUK_LIKELY(x)    __builtin_expect((x), 1)
#define DUK_UNLIKELY(x)  __builtin_expect((x), 0)
#endif
/* XXX: equivalent of clang __builtin_unpredictable? */

#if (defined(DUK_F_C99) || defined(DUK_F_CPP11)) && \
    defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 30101)
#define DUK_NOINLINE        __attribute__((noinline))
#define DUK_INLINE          inline
#define DUK_ALWAYS_INLINE   inline __attribute__((always_inline))
#endif

#if (defined(DUK_F_C99) || defined(DUK_F_CPP11)) && \
    defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 40300)
#define DUK_HOT             __attribute__((hot))
#define DUK_COLD            __attribute__((cold))
#endif

#if defined(DUK_F_DLL_BUILD) && defined(DUK_F_WINDOWS)
/* MSVC dllexport/dllimport: appropriate __declspec depends on whether we're
 * compiling Duktape or the application.
 */
#if defined(DUK_COMPILING_DUKTAPE)
#define DUK_EXTERNAL_DECL  extern __declspec(dllexport)
#define DUK_EXTERNAL       __declspec(dllexport)
#else
#define DUK_EXTERNAL_DECL  extern __declspec(dllimport)
#define DUK_EXTERNAL       should_not_happen
#endif
#if defined(DUK_SINGLE_FILE)
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#else
#define DUK_INTERNAL_DECL  extern
#define DUK_INTERNAL       /*empty*/
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static
#elif defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 40000)
#define DUK_EXTERNAL_DECL  __attribute__ ((visibility("default"))) extern
#define DUK_EXTERNAL       __attribute__ ((visibility("default")))
#if defined(DUK_SINGLE_FILE)
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
/* Minimize warnings for unused internal functions with GCC >= 3.1.1 and
 * Clang.  Based on documentation it should suffice to have the attribute
 * in the declaration only, but in practice some warnings are generated unless
 * the attribute is also applied to the definition.
 */
#define DUK_INTERNAL_DECL  static __attribute__ ((unused))
#define DUK_INTERNAL       static __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#endif
#else
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) __attribute__ ((unused)) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden"))) __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden")))
#endif
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static
#endif

#if defined(DUK_F_MINGW)
#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "mingw++"
#else
#define DUK_USE_COMPILER_STRING "mingw"
#endif
#elif defined(DUK_F_DJGPP)
#define DUK_USE_COMPILER_STRING "djgpp"
#else
#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "g++"
#else
#define DUK_USE_COMPILER_STRING "gcc"
#endif
#endif

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99) || (defined(DUK_F_CPP11) && defined(__GNUC__))
#define DUK_USE_VARIADIC_MACROS
#endif

#define DUK_USE_UNION_INITIALIZERS

#undef DUK_USE_FLEX_C99
#undef DUK_USE_FLEX_ZEROSIZE
#undef DUK_USE_FLEX_ONESIZE
#if defined(DUK_F_C99)
#define DUK_USE_FLEX_C99
#else
#define DUK_USE_FLEX_ZEROSIZE
#endif

/* Since 4.6 one can '#pragma GCC diagnostic push/pop'. */
#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 40600)
#define DUK_USE_GCC_PRAGMAS
#else
#undef DUK_USE_GCC_PRAGMAS
#endif

#define DUK_USE_PACK_GCC_ATTR

/* Availability varies based on platform (between GCC 4.4 and 4.8), and there
 * are apparently some bugs in GCC 4.x.  Check for GCC 5.0 before enabling
 * these to be safe.
 */
#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION >= 50000L)
#define DUK_BSWAP64(x) ((duk_uint64_t) __builtin_bswap64((duk_uint64_t) (x)))
#define DUK_BSWAP32(x) ((duk_uint32_t) __builtin_bswap32((duk_uint32_t) (x)))
#define DUK_BSWAP16(x) ((duk_uint16_t) __builtin_bswap16((duk_uint16_t) (x)))
#endif
#elif defined(DUK_F_MSVC)
/* --- MSVC --- */
/* http://msdn.microsoft.com/en-us/library/aa235362(VS.60).aspx */
#define DUK_NORETURN(decl)  __declspec(noreturn) decl

/* XXX: DUK_UNREACHABLE for msvc? */

#undef DUK_USE_BRANCH_HINTS

/* XXX: DUK_LIKELY, DUK_UNLIKELY for msvc? */

#if defined(DUK_F_DLL_BUILD) && defined(DUK_F_WINDOWS)
/* MSVC dllexport/dllimport: appropriate __declspec depends on whether we're
 * compiling Duktape or the application.
 */
#if defined(DUK_COMPILING_DUKTAPE)
#define DUK_EXTERNAL_DECL  extern __declspec(dllexport)
#define DUK_EXTERNAL       __declspec(dllexport)
#else
#define DUK_EXTERNAL_DECL  extern __declspec(dllimport)
#define DUK_EXTERNAL       should_not_happen
#endif
#if defined(DUK_SINGLE_FILE)
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#else
#define DUK_INTERNAL_DECL  extern
#define DUK_INTERNAL       /*empty*/
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static
#endif

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "msvc++"
#else
#define DUK_USE_COMPILER_STRING "msvc"
#endif

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99)
#define DUK_USE_VARIADIC_MACROS
#elif defined(_MSC_VER) && (_MSC_VER >= 1400)
/* VS2005+ should have variadic macros even when they're not C99. */
#define DUK_USE_VARIADIC_MACROS
#endif

#undef DUK_USE_UNION_INITIALIZERS
#if defined(_MSC_VER) && (_MSC_VER >= 1800)
/* VS2013+ supports union initializers but there's a bug involving union-inside-struct:
 * https://connect.microsoft.com/VisualStudio/feedback/details/805981
 * The bug was fixed (at least) in VS2015 so check for VS2015 for now:
 * https://blogs.msdn.microsoft.com/vcblog/2015/07/01/c-compiler-front-end-fixes-in-vs2015/
 * Manually tested using VS2013, CL reports 18.00.31101, so enable for VS2013 too.
 */
#define DUK_USE_UNION_INITIALIZERS
#endif

#undef DUK_USE_FLEX_C99
#undef DUK_USE_FLEX_ZEROSIZE
#undef DUK_USE_FLEX_ONESIZE
#if defined(DUK_F_C99)
#define DUK_USE_FLEX_C99
#else
#define DUK_USE_FLEX_ZEROSIZE
#endif

#undef DUK_USE_GCC_PRAGMAS

#define DUK_USE_PACK_MSVC_PRAGMA

/* These have been tested from VS2008 onwards; may work in older VS versions
 * too but not enabled by default.
 */
#if defined(_MSC_VER) && (_MSC_VER >= 1500)
#define DUK_NOINLINE        __declspec(noinline)
#define DUK_INLINE          __inline
#define DUK_ALWAYS_INLINE   __forceinline
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1900)
#define DUK_SNPRINTF     snprintf
#define DUK_VSNPRINTF    vsnprintf
#else
/* (v)snprintf() is missing before MSVC 2015.  Note that _(v)snprintf() does
 * NOT NUL terminate on truncation, but Duktape code never assumes that.
 * http://stackoverflow.com/questions/2915672/snprintf-and-visual-studio-2010
 */
#define DUK_SNPRINTF     _snprintf
#define DUK_VSNPRINTF    _vsnprintf
#endif

/* Avoid warning when doing DUK_UNREF(some_function). */
#if defined(_MSC_VER) && (_MSC_VER < 1500)
#pragma warning(disable: 4100 4101 4550 4551)
#define DUK_UNREF(x)
#else
#define DUK_UNREF(x)  do { __pragma(warning(suppress:4100 4101 4550 4551)) (x); } while (0)
#endif

/* Older versions of MSVC don't support the LL/ULL suffix. */
#define DUK_U64_CONSTANT(x) x##ui64
#define DUK_I64_CONSTANT(x) x##i64
#elif defined(DUK_F_EMSCRIPTEN)
/* --- Emscripten --- */
#define DUK_NORETURN(decl)  decl __attribute__((noreturn))

#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_unreachable)
#define DUK_UNREACHABLE()  do { __builtin_unreachable(); } while (0)
#endif
#endif

#define DUK_USE_BRANCH_HINTS
#define DUK_LIKELY(x)    __builtin_expect((x), 1)
#define DUK_UNLIKELY(x)  __builtin_expect((x), 0)
#if defined(__clang__) && defined(__has_builtin)
#if __has_builtin(__builtin_unpredictable)
#define DUK_UNPREDICTABLE(x)  __builtin_unpredictable((x))
#endif
#endif

#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_NOINLINE        __attribute__((noinline))
#define DUK_INLINE          inline
#define DUK_ALWAYS_INLINE   inline __attribute__((always_inline))
#endif

#define DUK_EXTERNAL_DECL  __attribute__ ((visibility("default"))) extern
#define DUK_EXTERNAL       __attribute__ ((visibility("default")))
#if defined(DUK_SINGLE_FILE)
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
/* Minimize warnings for unused internal functions with GCC >= 3.1.1 and
 * Clang.  Based on documentation it should suffice to have the attribute
 * in the declaration only, but in practice some warnings are generated unless
 * the attribute is also applied to the definition.
 */
#define DUK_INTERNAL_DECL  static __attribute__ ((unused))
#define DUK_INTERNAL       static __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  static
#define DUK_INTERNAL       static
#endif
#else
#if (defined(DUK_F_GCC_VERSION) && DUK_F_GCC_VERSION >= 30101) || defined(DUK_F_CLANG)
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) __attribute__ ((unused)) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden"))) __attribute__ ((unused))
#else
#define DUK_INTERNAL_DECL  __attribute__ ((visibility("hidden"))) extern
#define DUK_INTERNAL       __attribute__ ((visibility("hidden")))
#endif
#endif
#define DUK_LOCAL_DECL     static
#define DUK_LOCAL          static

#define DUK_USE_COMPILER_STRING "emscripten"

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_USE_VARIADIC_MACROS
#endif

#define DUK_USE_UNION_INITIALIZERS

#undef DUK_USE_FLEX_C99
#undef DUK_USE_FLEX_ZEROSIZE
#undef DUK_USE_FLEX_ONESIZE
#if defined(DUK_F_C99)
#define DUK_USE_FLEX_C99
#else
#define DUK_USE_FLEX_ZEROSIZE
#endif

#undef DUK_USE_GCC_PRAGMAS
#define DUK_USE_PACK_CLANG_ATTR
#elif defined(DUK_F_TINYC)
/* --- TinyC --- */
#undef DUK_USE_BRANCH_HINTS

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "tinyc++"
#else
#define DUK_USE_COMPILER_STRING "tinyc"
#endif

/* http://bellard.org/tcc/tcc-doc.html#SEC7 */
#define DUK_USE_VARIADIC_MACROS

#define DUK_USE_UNION_INITIALIZERS

/* Most portable, wastes space */
#define DUK_USE_FLEX_ONESIZE

/* Most portable, potentially wastes space */
#define DUK_USE_PACK_DUMMY_MEMBER
#elif defined(DUK_F_VBCC)
/* --- VBCC --- */
#undef DUK_USE_BRANCH_HINTS

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "vbcc-c++"
#else
#define DUK_USE_COMPILER_STRING "vbcc"
#endif

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_USE_VARIADIC_MACROS
#endif

/* VBCC supports C99 so check only for C99 for union initializer support.
 * Designated union initializers would possibly work even without a C99 check.
 */
#undef DUK_USE_UNION_INITIALIZERS
#if defined(DUK_F_C99)
#define DUK_USE_UNION_INITIALIZERS
#endif

#define DUK_USE_FLEX_ZEROSIZE
#define DUK_USE_PACK_DUMMY_MEMBER
#elif defined(DUK_F_BCC)
/* --- Bruce's C compiler --- */
#undef DUK_USE_BRANCH_HINTS

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "bcc++"
#else
#define DUK_USE_COMPILER_STRING "bcc"
#endif

/* Most portable */
#undef DUK_USE_VARIADIC_MACROS

/* Most portable, wastes space */
#undef DUK_USE_UNION_INITIALIZERS

/* Most portable, wastes space */
#define DUK_USE_FLEX_ONESIZE

/* Most portable, potentially wastes space */
#define DUK_USE_PACK_DUMMY_MEMBER

/* BCC, assume we're on x86. */
#if !defined(DUK_USE_BYTEORDER)
#define DUK_USE_BYTEORDER 1
#endif
#else
/* --- Generic --- */
#undef DUK_USE_BRANCH_HINTS

#if defined(DUK_F_CPP)
#define DUK_USE_COMPILER_STRING "generic-c++"
#else
#define DUK_USE_COMPILER_STRING "generic"
#endif

#undef DUK_USE_VARIADIC_MACROS
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_USE_VARIADIC_MACROS
#endif

/* C++ doesn't have standard designated union initializers ({ .foo = 1 }). */
#undef DUK_USE_UNION_INITIALIZERS
#if defined(DUK_F_C99)
#define DUK_USE_UNION_INITIALIZERS
#endif

/* Most portable, wastes space */
#define DUK_USE_FLEX_ONESIZE

/* Most portable, potentially wastes space */
#define DUK_USE_PACK_DUMMY_MEMBER
#endif  /*autodetect compiler*/

/* uclibc */
#if defined(__UCLIBC__)
#define DUK_F_UCLIBC
#endif

/*
 *  Wrapper typedefs and constants for integer types, also sanity check types.
 *
 *  C99 typedefs are quite good but not always available, and we want to avoid
 *  forcibly redefining the C99 typedefs.  So, there are Duktape wrappers for
 *  all C99 typedefs and Duktape code should only use these typedefs.  Type
 *  detection when C99 is not supported is best effort and may end up detecting
 *  some types incorrectly.
 *
 *  Pointer sizes are a portability problem: pointers to different types may
 *  have a different size and function pointers are very difficult to manage
 *  portably.
 *
 *  http://en.wikipedia.org/wiki/C_data_types#Fixed-width_integer_types
 *
 *  Note: there's an interesting corner case when trying to define minimum
 *  signed integer value constants which leads to the current workaround of
 *  defining e.g. -0x80000000 as (-0x7fffffffL - 1L).  See doc/code-issues.txt
 *  for a longer discussion.
 *
 *  Note: avoid typecasts and computations in macro integer constants as they
 *  can then no longer be used in macro relational expressions (such as
 *  #if DUK_SIZE_MAX < 0xffffffffUL).  There is internal code which relies on
 *  being able to compare DUK_SIZE_MAX against a limit.
 */

/* XXX: add feature options to force basic types from outside? */

#if !defined(INT_MAX)
#error INT_MAX not defined
#endif

/* Check that architecture is two's complement, standard C allows e.g.
 * INT_MIN to be -2**31+1 (instead of -2**31).
 */
#if defined(INT_MAX) && defined(INT_MIN)
#if INT_MAX != -(INT_MIN + 1)
#error platform does not seem complement of two
#endif
#else
#error cannot check complement of two
#endif

/* Pointer size determination based on __WORDSIZE or architecture when
 * that's not available.
 */
#if defined(DUK_F_X86) || defined(DUK_F_X32) || \
    defined(DUK_F_M68K) || defined(DUK_F_PPC32) || \
    defined(DUK_F_BCC) || \
    (defined(__WORDSIZE) && (__WORDSIZE == 32)) || \
    ((defined(DUK_F_OLD_SOLARIS) || defined(DUK_F_AIX) || \
      defined(DUK_F_HPUX)) && defined(_ILP32)) || \
    defined(DUK_F_ARM32)
#define DUK_F_32BIT_PTRS
#elif defined(DUK_F_X64) || \
      (defined(__WORDSIZE) && (__WORDSIZE == 64)) || \
   ((defined(DUK_F_OLD_SOLARIS) || defined(DUK_F_AIX) || \
     defined(DUK_F_HPUX)) && defined(_LP64)) || \
    defined(DUK_F_ARM64)
#define DUK_F_64BIT_PTRS
#else
/* not sure, not needed with C99 anyway */
#endif

/* Intermediate define for 'have inttypes.h' */
#undef DUK_F_HAVE_INTTYPES
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L) && \
    !(defined(DUK_F_AMIGAOS) && defined(DUK_F_VBCC))
/* vbcc + AmigaOS has C99 but no inttypes.h */
#define DUK_F_HAVE_INTTYPES
#elif defined(__cplusplus) && (__cplusplus >= 201103L)
/* C++11 apparently ratified stdint.h */
#define DUK_F_HAVE_INTTYPES
#endif

/* Basic integer typedefs and limits, preferably from inttypes.h, otherwise
 * through automatic detection.
 */
#if defined(DUK_F_HAVE_INTTYPES)
/* C99 or compatible */

#define DUK_F_HAVE_64BIT
#include <inttypes.h>

typedef uint8_t duk_uint8_t;
typedef int8_t duk_int8_t;
typedef uint16_t duk_uint16_t;
typedef int16_t duk_int16_t;
typedef uint32_t duk_uint32_t;
typedef int32_t duk_int32_t;
typedef uint64_t duk_uint64_t;
typedef int64_t duk_int64_t;
typedef uint_least8_t duk_uint_least8_t;
typedef int_least8_t duk_int_least8_t;
typedef uint_least16_t duk_uint_least16_t;
typedef int_least16_t duk_int_least16_t;
typedef uint_least32_t duk_uint_least32_t;
typedef int_least32_t duk_int_least32_t;
typedef uint_least64_t duk_uint_least64_t;
typedef int_least64_t duk_int_least64_t;
typedef uint_fast8_t duk_uint_fast8_t;
typedef int_fast8_t duk_int_fast8_t;
typedef uint_fast16_t duk_uint_fast16_t;
typedef int_fast16_t duk_int_fast16_t;
typedef uint_fast32_t duk_uint_fast32_t;
typedef int_fast32_t duk_int_fast32_t;
typedef uint_fast64_t duk_uint_fast64_t;
typedef int_fast64_t duk_int_fast64_t;
typedef uintptr_t duk_uintptr_t;
typedef intptr_t duk_intptr_t;
typedef uintmax_t duk_uintmax_t;
typedef intmax_t duk_intmax_t;

#define DUK_UINT8_MIN         0
#define DUK_UINT8_MAX         UINT8_MAX
#define DUK_INT8_MIN          INT8_MIN
#define DUK_INT8_MAX          INT8_MAX
#define DUK_UINT_LEAST8_MIN   0
#define DUK_UINT_LEAST8_MAX   UINT_LEAST8_MAX
#define DUK_INT_LEAST8_MIN    INT_LEAST8_MIN
#define DUK_INT_LEAST8_MAX    INT_LEAST8_MAX
#define DUK_UINT_FAST8_MIN    0
#define DUK_UINT_FAST8_MAX    UINT_FAST8_MAX
#define DUK_INT_FAST8_MIN     INT_FAST8_MIN
#define DUK_INT_FAST8_MAX     INT_FAST8_MAX
#define DUK_UINT16_MIN        0
#define DUK_UINT16_MAX        UINT16_MAX
#define DUK_INT16_MIN         INT16_MIN
#define DUK_INT16_MAX         INT16_MAX
#define DUK_UINT_LEAST16_MIN  0
#define DUK_UINT_LEAST16_MAX  UINT_LEAST16_MAX
#define DUK_INT_LEAST16_MIN   INT_LEAST16_MIN
#define DUK_INT_LEAST16_MAX   INT_LEAST16_MAX
#define DUK_UINT_FAST16_MIN   0
#define DUK_UINT_FAST16_MAX   UINT_FAST16_MAX
#define DUK_INT_FAST16_MIN    INT_FAST16_MIN
#define DUK_INT_FAST16_MAX    INT_FAST16_MAX
#define DUK_UINT32_MIN        0
#define DUK_UINT32_MAX        UINT32_MAX
#define DUK_INT32_MIN         INT32_MIN
#define DUK_INT32_MAX         INT32_MAX
#define DUK_UINT_LEAST32_MIN  0
#define DUK_UINT_LEAST32_MAX  UINT_LEAST32_MAX
#define DUK_INT_LEAST32_MIN   INT_LEAST32_MIN
#define DUK_INT_LEAST32_MAX   INT_LEAST32_MAX
#define DUK_UINT_FAST32_MIN   0
#define DUK_UINT_FAST32_MAX   UINT_FAST32_MAX
#define DUK_INT_FAST32_MIN    INT_FAST32_MIN
#define DUK_INT_FAST32_MAX    INT_FAST32_MAX
#define DUK_UINT64_MIN        0
#define DUK_UINT64_MAX        UINT64_MAX
#define DUK_INT64_MIN         INT64_MIN
#define DUK_INT64_MAX         INT64_MAX
#define DUK_UINT_LEAST64_MIN  0
#define DUK_UINT_LEAST64_MAX  UINT_LEAST64_MAX
#define DUK_INT_LEAST64_MIN   INT_LEAST64_MIN
#define DUK_INT_LEAST64_MAX   INT_LEAST64_MAX
#define DUK_UINT_FAST64_MIN   0
#define DUK_UINT_FAST64_MAX   UINT_FAST64_MAX
#define DUK_INT_FAST64_MIN    INT_FAST64_MIN
#define DUK_INT_FAST64_MAX    INT_FAST64_MAX

#define DUK_UINTPTR_MIN       0
#define DUK_UINTPTR_MAX       UINTPTR_MAX
#define DUK_INTPTR_MIN        INTPTR_MIN
#define DUK_INTPTR_MAX        INTPTR_MAX
#undef DUK_UINTPTR_MAX_COMPUTED

#define DUK_UINTMAX_MIN       0
#define DUK_UINTMAX_MAX       UINTMAX_MAX
#define DUK_INTMAX_MIN        INTMAX_MIN
#define DUK_INTMAX_MAX        INTMAX_MAX

#define DUK_SIZE_MIN          0
#define DUK_SIZE_MAX          SIZE_MAX
#undef DUK_SIZE_MAX_COMPUTED

#else  /* C99 types */

/* When C99 types are not available, we use heuristic detection to get
 * the basic 8, 16, 32, and (possibly) 64 bit types.  The fast/least
 * types are then assumed to be exactly the same for now: these could
 * be improved per platform but C99 types are very often now available.
 * 64-bit types are not available on all platforms; this is OK at least
 * on 32-bit platforms.
 *
 * This detection code is necessarily a bit hacky and can provide typedefs
 * and defines that won't work correctly on some exotic platform.
 */

#if (defined(CHAR_BIT) && (CHAR_BIT == 8)) || \
    (defined(UCHAR_MAX) && (UCHAR_MAX == 255))
typedef unsigned char duk_uint8_t;
typedef signed char duk_int8_t;
#else
#error cannot detect 8-bit type
#endif

#if defined(USHRT_MAX) && (USHRT_MAX == 65535UL)
typedef unsigned short duk_uint16_t;
typedef signed short duk_int16_t;
#elif defined(UINT_MAX) && (UINT_MAX == 65535UL)
/* On some platforms int is 16-bit but long is 32-bit (e.g. PureC) */
typedef unsigned int duk_uint16_t;
typedef signed int duk_int16_t;
#else
#error cannot detect 16-bit type
#endif

#if defined(UINT_MAX) && (UINT_MAX == 4294967295UL)
typedef unsigned int duk_uint32_t;
typedef signed int duk_int32_t;
#elif defined(ULONG_MAX) && (ULONG_MAX == 4294967295UL)
/* On some platforms int is 16-bit but long is 32-bit (e.g. PureC) */
typedef unsigned long duk_uint32_t;
typedef signed long duk_int32_t;
#else
#error cannot detect 32-bit type
#endif

/* 64-bit type detection is a bit tricky.
 *
 * ULLONG_MAX is a standard define.  __LONG_LONG_MAX__ and __ULONG_LONG_MAX__
 * are used by at least GCC (even if system headers don't provide ULLONG_MAX).
 * Some GCC variants may provide __LONG_LONG_MAX__ but not __ULONG_LONG_MAX__.
 *
 * ULL / LL constants are rejected / warned about by some compilers, even if
 * the compiler has a 64-bit type and the compiler/system headers provide an
 * unsupported constant (ULL/LL)!  Try to avoid using ULL / LL constants.
 * As a side effect we can only check that e.g. ULONG_MAX is larger than 32
 * bits but can't be sure it is exactly 64 bits.  Self tests will catch such
 * cases.
 */
#undef DUK_F_HAVE_64BIT
#if !defined(DUK_F_HAVE_64BIT) && defined(ULONG_MAX)
#if (ULONG_MAX > 4294967295UL)
#define DUK_F_HAVE_64BIT
typedef unsigned long duk_uint64_t;
typedef signed long duk_int64_t;
#endif
#endif
#if !defined(DUK_F_HAVE_64BIT) && defined(ULLONG_MAX)
#if (ULLONG_MAX > 4294967295UL)
#define DUK_F_HAVE_64BIT
typedef unsigned long long duk_uint64_t;
typedef signed long long duk_int64_t;
#endif
#endif
#if !defined(DUK_F_HAVE_64BIT) && defined(__ULONG_LONG_MAX__)
#if (__ULONG_LONG_MAX__ > 4294967295UL)
#define DUK_F_HAVE_64BIT
typedef unsigned long long duk_uint64_t;
typedef signed long long duk_int64_t;
#endif
#endif
#if !defined(DUK_F_HAVE_64BIT) && defined(__LONG_LONG_MAX__)
#if (__LONG_LONG_MAX__ > 2147483647L)
#define DUK_F_HAVE_64BIT
typedef unsigned long long duk_uint64_t;
typedef signed long long duk_int64_t;
#endif
#endif
#if !defined(DUK_F_HAVE_64BIT) && defined(DUK_F_MINGW)
#define DUK_F_HAVE_64BIT
typedef unsigned long duk_uint64_t;
typedef signed long duk_int64_t;
#endif
#if !defined(DUK_F_HAVE_64BIT) && defined(DUK_F_MSVC)
#define DUK_F_HAVE_64BIT
typedef unsigned __int64 duk_uint64_t;
typedef signed __int64 duk_int64_t;
#endif
#if !defined(DUK_F_HAVE_64BIT)
/* cannot detect 64-bit type, not always needed so don't error */
#endif

typedef duk_uint8_t duk_uint_least8_t;
typedef duk_int8_t duk_int_least8_t;
typedef duk_uint16_t duk_uint_least16_t;
typedef duk_int16_t duk_int_least16_t;
typedef duk_uint32_t duk_uint_least32_t;
typedef duk_int32_t duk_int_least32_t;
typedef duk_uint8_t duk_uint_fast8_t;
typedef duk_int8_t duk_int_fast8_t;
typedef duk_uint16_t duk_uint_fast16_t;
typedef duk_int16_t duk_int_fast16_t;
typedef duk_uint32_t duk_uint_fast32_t;
typedef duk_int32_t duk_int_fast32_t;
#if defined(DUK_F_HAVE_64BIT)
typedef duk_uint64_t duk_uint_least64_t;
typedef duk_int64_t duk_int_least64_t;
typedef duk_uint64_t duk_uint_fast64_t;
typedef duk_int64_t duk_int_fast64_t;
#endif
#if defined(DUK_F_HAVE_64BIT)
typedef duk_uint64_t duk_uintmax_t;
typedef duk_int64_t duk_intmax_t;
#else
typedef duk_uint32_t duk_uintmax_t;
typedef duk_int32_t duk_intmax_t;
#endif

/* Note: the funny looking computations for signed minimum 16-bit, 32-bit, and
 * 64-bit values are intentional as the obvious forms (e.g. -0x80000000L) are
 * -not- portable.  See code-issues.txt for a detailed discussion.
 */
#define DUK_UINT8_MIN         0UL
#define DUK_UINT8_MAX         0xffUL
#define DUK_INT8_MIN          (-0x80L)
#define DUK_INT8_MAX          0x7fL
#define DUK_UINT_LEAST8_MIN   0UL
#define DUK_UINT_LEAST8_MAX   0xffUL
#define DUK_INT_LEAST8_MIN    (-0x80L)
#define DUK_INT_LEAST8_MAX    0x7fL
#define DUK_UINT_FAST8_MIN    0UL
#define DUK_UINT_FAST8_MAX    0xffUL
#define DUK_INT_FAST8_MIN     (-0x80L)
#define DUK_INT_FAST8_MAX     0x7fL
#define DUK_UINT16_MIN        0UL
#define DUK_UINT16_MAX        0xffffUL
#define DUK_INT16_MIN         (-0x7fffL - 1L)
#define DUK_INT16_MAX         0x7fffL
#define DUK_UINT_LEAST16_MIN  0UL
#define DUK_UINT_LEAST16_MAX  0xffffUL
#define DUK_INT_LEAST16_MIN   (-0x7fffL - 1L)
#define DUK_INT_LEAST16_MAX   0x7fffL
#define DUK_UINT_FAST16_MIN   0UL
#define DUK_UINT_FAST16_MAX   0xffffUL
#define DUK_INT_FAST16_MIN    (-0x7fffL - 1L)
#define DUK_INT_FAST16_MAX    0x7fffL
#define DUK_UINT32_MIN        0UL
#define DUK_UINT32_MAX        0xffffffffUL
#define DUK_INT32_MIN         (-0x7fffffffL - 1L)
#define DUK_INT32_MAX         0x7fffffffL
#define DUK_UINT_LEAST32_MIN  0UL
#define DUK_UINT_LEAST32_MAX  0xffffffffUL
#define DUK_INT_LEAST32_MIN   (-0x7fffffffL - 1L)
#define DUK_INT_LEAST32_MAX   0x7fffffffL
#define DUK_UINT_FAST32_MIN   0UL
#define DUK_UINT_FAST32_MAX   0xffffffffUL
#define DUK_INT_FAST32_MIN    (-0x7fffffffL - 1L)
#define DUK_INT_FAST32_MAX    0x7fffffffL

/* 64-bit constants.  Since LL / ULL constants are not always available,
 * use computed values.  These values can't be used in preprocessor
 * comparisons; flag them as such.
 */
#if defined(DUK_F_HAVE_64BIT)
#define DUK_UINT64_MIN        ((duk_uint64_t) 0)
#define DUK_UINT64_MAX        ((duk_uint64_t) -1)
#define DUK_INT64_MIN         ((duk_int64_t) (~(DUK_UINT64_MAX >> 1)))
#define DUK_INT64_MAX         ((duk_int64_t) (DUK_UINT64_MAX >> 1))
#define DUK_UINT_LEAST64_MIN  DUK_UINT64_MIN
#define DUK_UINT_LEAST64_MAX  DUK_UINT64_MAX
#define DUK_INT_LEAST64_MIN   DUK_INT64_MIN
#define DUK_INT_LEAST64_MAX   DUK_INT64_MAX
#define DUK_UINT_FAST64_MIN   DUK_UINT64_MIN
#define DUK_UINT_FAST64_MAX   DUK_UINT64_MAX
#define DUK_INT_FAST64_MIN    DUK_INT64_MIN
#define DUK_INT_FAST64_MAX    DUK_INT64_MAX
#define DUK_UINT64_MIN_COMPUTED
#define DUK_UINT64_MAX_COMPUTED
#define DUK_INT64_MIN_COMPUTED
#define DUK_INT64_MAX_COMPUTED
#define DUK_UINT_LEAST64_MIN_COMPUTED
#define DUK_UINT_LEAST64_MAX_COMPUTED
#define DUK_INT_LEAST64_MIN_COMPUTED
#define DUK_INT_LEAST64_MAX_COMPUTED
#define DUK_UINT_FAST64_MIN_COMPUTED
#define DUK_UINT_FAST64_MAX_COMPUTED
#define DUK_INT_FAST64_MIN_COMPUTED
#define DUK_INT_FAST64_MAX_COMPUTED
#endif

#if defined(DUK_F_HAVE_64BIT)
#define DUK_UINTMAX_MIN       DUK_UINT64_MIN
#define DUK_UINTMAX_MAX       DUK_UINT64_MAX
#define DUK_INTMAX_MIN        DUK_INT64_MIN
#define DUK_INTMAX_MAX        DUK_INT64_MAX
#define DUK_UINTMAX_MIN_COMPUTED
#define DUK_UINTMAX_MAX_COMPUTED
#define DUK_INTMAX_MIN_COMPUTED
#define DUK_INTMAX_MAX_COMPUTED
#else
#define DUK_UINTMAX_MIN       0UL
#define DUK_UINTMAX_MAX       0xffffffffUL
#define DUK_INTMAX_MIN        (-0x7fffffffL - 1L)
#define DUK_INTMAX_MAX        0x7fffffffL
#endif

/* This detection is not very reliable. */
#if defined(DUK_F_32BIT_PTRS)
typedef duk_int32_t duk_intptr_t;
typedef duk_uint32_t duk_uintptr_t;
#define DUK_UINTPTR_MIN       DUK_UINT32_MIN
#define DUK_UINTPTR_MAX       DUK_UINT32_MAX
#define DUK_INTPTR_MIN        DUK_INT32_MIN
#define DUK_INTPTR_MAX        DUK_INT32_MAX
#elif defined(DUK_F_64BIT_PTRS) && defined(DUK_F_HAVE_64BIT)
typedef duk_int64_t duk_intptr_t;
typedef duk_uint64_t duk_uintptr_t;
#define DUK_UINTPTR_MIN       DUK_UINT64_MIN
#define DUK_UINTPTR_MAX       DUK_UINT64_MAX
#define DUK_INTPTR_MIN        DUK_INT64_MIN
#define DUK_INTPTR_MAX        DUK_INT64_MAX
#define DUK_UINTPTR_MIN_COMPUTED
#define DUK_UINTPTR_MAX_COMPUTED
#define DUK_INTPTR_MIN_COMPUTED
#define DUK_INTPTR_MAX_COMPUTED
#else
#error cannot determine intptr type
#endif

/* SIZE_MAX may be missing so use an approximate value for it. */
#undef DUK_SIZE_MAX_COMPUTED
#if !defined(SIZE_MAX)
#define DUK_SIZE_MAX_COMPUTED
#define SIZE_MAX              ((size_t) (-1))
#endif
#define DUK_SIZE_MIN          0
#define DUK_SIZE_MAX          SIZE_MAX

#endif  /* C99 types */

/* A few types are assumed to always exist. */
typedef size_t duk_size_t;
typedef ptrdiff_t duk_ptrdiff_t;

/* The best type for an "all around int" in Duktape internals is "at least
 * 32 bit signed integer" which is most convenient.  Same for unsigned type.
 * Prefer 'int' when large enough, as it is almost always a convenient type.
 */
#if defined(UINT_MAX) && (UINT_MAX >= 0xffffffffUL)
typedef int duk_int_t;
typedef unsigned int duk_uint_t;
#define DUK_INT_MIN           INT_MIN
#define DUK_INT_MAX           INT_MAX
#define DUK_UINT_MIN          0
#define DUK_UINT_MAX          UINT_MAX
#else
typedef duk_int_fast32_t duk_int_t;
typedef duk_uint_fast32_t duk_uint_t;
#define DUK_INT_MIN           DUK_INT_FAST32_MIN
#define DUK_INT_MAX           DUK_INT_FAST32_MAX
#define DUK_UINT_MIN          DUK_UINT_FAST32_MIN
#define DUK_UINT_MAX          DUK_UINT_FAST32_MAX
#endif

/* Same as 'duk_int_t' but guaranteed to be a 'fast' variant if this
 * distinction matters for the CPU.  These types are used mainly in the
 * executor where it might really matter.
 */
typedef duk_int_fast32_t duk_int_fast_t;
typedef duk_uint_fast32_t duk_uint_fast_t;
#define DUK_INT_FAST_MIN      DUK_INT_FAST32_MIN
#define DUK_INT_FAST_MAX      DUK_INT_FAST32_MAX
#define DUK_UINT_FAST_MIN     DUK_UINT_FAST32_MIN
#define DUK_UINT_FAST_MAX     DUK_UINT_FAST32_MAX

/* Small integers (16 bits or more) can fall back to the 'int' type, but
 * have a typedef so they are marked "small" explicitly.
 */
typedef int duk_small_int_t;
typedef unsigned int duk_small_uint_t;
#define DUK_SMALL_INT_MIN     INT_MIN
#define DUK_SMALL_INT_MAX     INT_MAX
#define DUK_SMALL_UINT_MIN    0
#define DUK_SMALL_UINT_MAX    UINT_MAX

/* Fast variants of small integers, again for really fast paths like the
 * executor.
 */
typedef duk_int_fast16_t duk_small_int_fast_t;
typedef duk_uint_fast16_t duk_small_uint_fast_t;
#define DUK_SMALL_INT_FAST_MIN    DUK_INT_FAST16_MIN
#define DUK_SMALL_INT_FAST_MAX    DUK_INT_FAST16_MAX
#define DUK_SMALL_UINT_FAST_MIN   DUK_UINT_FAST16_MIN
#define DUK_SMALL_UINT_FAST_MAX   DUK_UINT_FAST16_MAX

/* Boolean values are represented with the platform 'unsigned int'. */
typedef duk_small_uint_t duk_bool_t;
#define DUK_BOOL_MIN              DUK_SMALL_UINT_MIN
#define DUK_BOOL_MAX              DUK_SMALL_UINT_MAX

/* Signed boolean is useful in some internal use cases where negative
 * values can e.g. indicate "not found".
 */
typedef duk_small_int_t duk_sbool_t;
#define DUK_SBOOL_MIN             DUK_SMALL_INT_MIN
#define DUK_SBOOL_MAX             DUK_SMALL_INT_MAX

/* Index values must have at least 32-bit signed range. */
typedef duk_int_t duk_idx_t;
#define DUK_IDX_MIN               DUK_INT_MIN
#define DUK_IDX_MAX               DUK_INT_MAX

/* Unsigned index variant. */
typedef duk_uint_t duk_uidx_t;
#define DUK_UIDX_MIN              DUK_UINT_MIN
#define DUK_UIDX_MAX              DUK_UINT_MAX

/* Array index values, could be exact 32 bits.
 * Currently no need for signed duk_arridx_t.
 */
typedef duk_uint_t duk_uarridx_t;
#define DUK_UARRIDX_MIN           DUK_UINT_MIN
#define DUK_UARRIDX_MAX           DUK_UINT_MAX

/* Duktape/C function return value, platform int is enough for now to
 * represent 0, 1, or negative error code.  Must be compatible with
 * assigning truth values (e.g. duk_ret_t rc = (foo == bar);).
 */
typedef duk_small_int_t duk_ret_t;
#define DUK_RET_MIN               DUK_SMALL_INT_MIN
#define DUK_RET_MAX               DUK_SMALL_INT_MAX

/* Error codes are represented with platform int.  High bits are used
 * for flags and such, so 32 bits are needed.
 */
typedef duk_int_t duk_errcode_t;
#define DUK_ERRCODE_MIN           DUK_INT_MIN
#define DUK_ERRCODE_MAX           DUK_INT_MAX

/* Codepoint type.  Must be 32 bits or more because it is used also for
 * internal codepoints.  The type is signed because negative codepoints
 * are used as internal markers (e.g. to mark EOF or missing argument).
 * (X)UTF-8/CESU-8 encode/decode take and return an unsigned variant to
 * ensure duk_uint32_t casts back and forth nicely.  Almost everything
 * else uses the signed one.
 */
typedef duk_int_t duk_codepoint_t;
typedef duk_uint_t duk_ucodepoint_t;
#define DUK_CODEPOINT_MIN         DUK_INT_MIN
#define DUK_CODEPOINT_MAX         DUK_INT_MAX
#define DUK_UCODEPOINT_MIN        DUK_UINT_MIN
#define DUK_UCODEPOINT_MAX        DUK_UINT_MAX

/* IEEE float/double typedef. */
typedef float duk_float_t;
typedef double duk_double_t;

/* We're generally assuming that we're working on a platform with a 32-bit
 * address space.  If DUK_SIZE_MAX is a typecast value (which is necessary
 * if SIZE_MAX is missing), the check must be avoided because the
 * preprocessor can't do a comparison.
 */
#if !defined(DUK_SIZE_MAX)
#error DUK_SIZE_MAX is undefined, probably missing SIZE_MAX
#elif !defined(DUK_SIZE_MAX_COMPUTED)
#if DUK_SIZE_MAX < 0xffffffffUL
/* On some systems SIZE_MAX can be smaller than max unsigned 32-bit value
 * which seems incorrect if size_t is (at least) an unsigned 32-bit type.
 * However, it doesn't seem useful to error out compilation if this is the
 * case.
 */
#endif
#endif

/* Type used in public API declarations and user code.  Typedef maps to
 * 'struct duk_hthread' like the 'duk_hthread' typedef which is used
 * exclusively in internals.
 */
typedef struct duk_hthread duk_context;

/* Check whether we should use 64-bit integers or not.
 *
 * Quite incomplete now.  Use 64-bit types if detected (C99 or other detection)
 * unless they are known to be unreliable.  For instance, 64-bit types are
 * available on VBCC but seem to misbehave.
 */
#if defined(DUK_F_HAVE_64BIT) && !defined(DUK_F_VBCC)
#define DUK_USE_64BIT_OPS
#else
#undef DUK_USE_64BIT_OPS
#endif

/*
 *  Fill-ins for platform, architecture, and compiler
 */

/* An abort()-like primitive is needed by the default fatal error handler. */
#if !defined(DUK_ABORT)
#define DUK_ABORT             abort
#endif

#if !defined(DUK_SETJMP)
#define DUK_JMPBUF_TYPE       jmp_buf
#define DUK_SETJMP(jb)        setjmp((jb))
#define DUK_LONGJMP(jb)       longjmp((jb), 1)
#endif

#if 0
/* sigsetjmp() alternative */
#define DUK_JMPBUF_TYPE       sigjmp_buf
#define DUK_SETJMP(jb)        sigsetjmp((jb))
#define DUK_LONGJMP(jb)       siglongjmp((jb), 1)
#endif

/* Special naming to avoid conflict with e.g. DUK_FREE() in duk_heap.h
 * (which is unfortunately named).  May sometimes need replacement, e.g.
 * some compilers don't handle zero length or NULL correctly in realloc().
 */
#if !defined(DUK_ANSI_MALLOC)
#define DUK_ANSI_MALLOC      malloc
#endif
#if !defined(DUK_ANSI_REALLOC)
#define DUK_ANSI_REALLOC     realloc
#endif
#if !defined(DUK_ANSI_CALLOC)
#define DUK_ANSI_CALLOC      calloc
#endif
#if !defined(DUK_ANSI_FREE)
#define DUK_ANSI_FREE        free
#endif

/* ANSI C (various versions) and some implementations require that the
 * pointer arguments to memset(), memcpy(), and memmove() be valid values
 * even when byte size is 0 (even a NULL pointer is considered invalid in
 * this context).  Zero-size operations as such are allowed, as long as their
 * pointer arguments point to a valid memory area.  The DUK_MEMSET(),
 * DUK_MEMCPY(), and DUK_MEMMOVE() macros require this same behavior, i.e.:
 * (1) pointers must be valid and non-NULL, (2) zero size must otherwise be
 * allowed.  If these are not fulfilled, a macro wrapper is needed.
 *
 *   http://stackoverflow.com/questions/5243012/is-it-guaranteed-to-be-safe-to-perform-memcpy0-0-0
 *   http://lists.cs.uiuc.edu/pipermail/llvmdev/2007-October/011065.html
 *
 * Not sure what's the required behavior when a pointer points just past the
 * end of a buffer, which often happens in practice (e.g. zero size memmoves).
 * For example, if allocation size is 3, the following pointer would not
 * technically point to a valid memory byte:
 *
 *   <-- alloc -->
 *   | 0 | 1 | 2 | .....
 *                 ^-- p=3, points after last valid byte (2)
 */
#if !defined(DUK_MEMCPY)
#if defined(DUK_F_UCLIBC)
/* Old uclibcs have a broken memcpy so use memmove instead (this is overly wide
 * now on purpose): http://lists.uclibc.org/pipermail/uclibc-cvs/2008-October/025511.html
 */
#define DUK_MEMCPY       memmove
#else
#define DUK_MEMCPY       memcpy
#endif
#endif
#if !defined(DUK_MEMMOVE)
#define DUK_MEMMOVE      memmove
#endif
#if !defined(DUK_MEMCMP)
#define DUK_MEMCMP       memcmp
#endif
#if !defined(DUK_MEMSET)
#define DUK_MEMSET       memset
#endif
#if !defined(DUK_STRLEN)
#define DUK_STRLEN       strlen
#endif
#if !defined(DUK_STRCMP)
#define DUK_STRCMP       strcmp
#endif
#if !defined(DUK_STRNCMP)
#define DUK_STRNCMP      strncmp
#endif
#if !defined(DUK_SPRINTF)
#define DUK_SPRINTF      sprintf
#endif
#if !defined(DUK_SNPRINTF)
/* snprintf() is technically not part of C89 but usually available. */
#define DUK_SNPRINTF     snprintf
#endif
#if !defined(DUK_VSPRINTF)
#define DUK_VSPRINTF     vsprintf
#endif
#if !defined(DUK_VSNPRINTF)
/* vsnprintf() is technically not part of C89 but usually available. */
#define DUK_VSNPRINTF    vsnprintf
#endif
#if !defined(DUK_SSCANF)
#define DUK_SSCANF       sscanf
#endif
#if !defined(DUK_VSSCANF)
#define DUK_VSSCANF      vsscanf
#endif
#if !defined(DUK_MEMZERO)
#define DUK_MEMZERO(p,n) DUK_MEMSET((p), 0, (n))
#endif

#if !defined(DUK_DOUBLE_INFINITY)
#undef DUK_USE_COMPUTED_INFINITY
#if defined(DUK_F_GCC_VERSION) && (DUK_F_GCC_VERSION < 40600)
/* GCC older than 4.6: avoid overflow warnings related to using INFINITY */
#define DUK_DOUBLE_INFINITY  (__builtin_inf())
#elif defined(INFINITY)
#define DUK_DOUBLE_INFINITY  ((double) INFINITY)
#elif !defined(DUK_F_VBCC) && !defined(DUK_F_MSVC) && !defined(DUK_F_BCC) && \
      !defined(DUK_F_OLD_SOLARIS) && !defined(DUK_F_AIX)
#define DUK_DOUBLE_INFINITY  (1.0 / 0.0)
#else
/* In VBCC (1.0 / 0.0) results in a warning and 0.0 instead of infinity.
 * Use a computed infinity (initialized when a heap is created at the
 * latest).
 */
#define DUK_USE_COMPUTED_INFINITY
#define DUK_DOUBLE_INFINITY  duk_computed_infinity
#endif
#endif

#if !defined(DUK_DOUBLE_NAN)
#undef DUK_USE_COMPUTED_NAN
#if defined(NAN)
#define DUK_DOUBLE_NAN       NAN
#elif !defined(DUK_F_VBCC) && !defined(DUK_F_MSVC) && !defined(DUK_F_BCC) && \
      !defined(DUK_F_OLD_SOLARIS) && !defined(DUK_F_AIX)
#define DUK_DOUBLE_NAN       (0.0 / 0.0)
#else
/* In VBCC (0.0 / 0.0) results in a warning and 0.0 instead of NaN.
 * In MSVC (VS2010 Express) (0.0 / 0.0) results in a compile error.
 * Use a computed NaN (initialized when a heap is created at the
 * latest).
 */
#define DUK_USE_COMPUTED_NAN
#define DUK_DOUBLE_NAN       duk_computed_nan
#endif
#endif

/* Many platforms are missing fpclassify() and friends, so use replacements
 * if necessary.  The replacement constants (FP_NAN etc) can be anything but
 * match Linux constants now.
 */
#undef DUK_USE_REPL_FPCLASSIFY
#undef DUK_USE_REPL_SIGNBIT
#undef DUK_USE_REPL_ISFINITE
#undef DUK_USE_REPL_ISNAN
#undef DUK_USE_REPL_ISINF

/* Complex condition broken into separate parts. */
#undef DUK_F_USE_REPL_ALL
#if !(defined(FP_NAN) && defined(FP_INFINITE) && defined(FP_ZERO) && \
      defined(FP_SUBNORMAL) && defined(FP_NORMAL))
/* Missing some obvious constants. */
#define DUK_F_USE_REPL_ALL
#elif defined(DUK_F_AMIGAOS) && defined(DUK_F_VBCC)
/* VBCC is missing the built-ins even in C99 mode (perhaps a header issue). */
#define DUK_F_USE_REPL_ALL
#elif defined(DUK_F_AMIGAOS) && defined(DUK_F_M68K)
/* AmigaOS + M68K seems to have math issues even when using GCC cross
 * compilation.  Use replacements for all AmigaOS versions on M68K
 * regardless of compiler.
 */
#define DUK_F_USE_REPL_ALL
#elif defined(DUK_F_FREEBSD) && defined(DUK_F_CLANG)
/* Placeholder fix for (detection is wider than necessary):
 * http://llvm.org/bugs/show_bug.cgi?id=17788
 */
#define DUK_F_USE_REPL_ALL
#elif defined(DUK_F_UCLIBC)
/* At least some uclibc versions have broken floating point math.  For
 * example, fpclassify() can incorrectly classify certain NaN formats.
 * To be safe, use replacements.
 */
#define DUK_F_USE_REPL_ALL
#elif defined(DUK_F_AIX)
/* Older versions may be missing isnan(), etc. */
#define DUK_F_USE_REPL_ALL
#endif

#if defined(DUK_F_USE_REPL_ALL)
#define DUK_USE_REPL_FPCLASSIFY
#define DUK_USE_REPL_SIGNBIT
#define DUK_USE_REPL_ISFINITE
#define DUK_USE_REPL_ISNAN
#define DUK_USE_REPL_ISINF
#define DUK_FPCLASSIFY       duk_repl_fpclassify
#define DUK_SIGNBIT          duk_repl_signbit
#define DUK_ISFINITE         duk_repl_isfinite
#define DUK_ISNAN            duk_repl_isnan
#define DUK_ISINF            duk_repl_isinf
#define DUK_FP_NAN           0
#define DUK_FP_INFINITE      1
#define DUK_FP_ZERO          2
#define DUK_FP_SUBNORMAL     3
#define DUK_FP_NORMAL        4
#else
#define DUK_FPCLASSIFY       fpclassify
#define DUK_SIGNBIT          signbit
#define DUK_ISFINITE         isfinite
#define DUK_ISNAN            isnan
#define DUK_ISINF            isinf
#define DUK_FP_NAN           FP_NAN
#define DUK_FP_INFINITE      FP_INFINITE
#define DUK_FP_ZERO          FP_ZERO
#define DUK_FP_SUBNORMAL     FP_SUBNORMAL
#define DUK_FP_NORMAL        FP_NORMAL
#endif

#if defined(DUK_F_USE_REPL_ALL)
#undef DUK_F_USE_REPL_ALL
#endif

/* These functions don't currently need replacement but are wrapped for
 * completeness.  Because these are used as function pointers, they need
 * to be defined as concrete C functions (not macros).
 */
#if !defined(DUK_FABS)
#define DUK_FABS             fabs
#endif
#if !defined(DUK_FLOOR)
#define DUK_FLOOR            floor
#endif
#if !defined(DUK_CEIL)
#define DUK_CEIL             ceil
#endif
#if !defined(DUK_FMOD)
#define DUK_FMOD             fmod
#endif
#if !defined(DUK_POW)
#define DUK_POW              pow
#endif
#if !defined(DUK_ACOS)
#define DUK_ACOS             acos
#endif
#if !defined(DUK_ASIN)
#define DUK_ASIN             asin
#endif
#if !defined(DUK_ATAN)
#define DUK_ATAN             atan
#endif
#if !defined(DUK_ATAN2)
#define DUK_ATAN2            atan2
#endif
#if !defined(DUK_SIN)
#define DUK_SIN              sin
#endif
#if !defined(DUK_COS)
#define DUK_COS              cos
#endif
#if !defined(DUK_TAN)
#define DUK_TAN              tan
#endif
#if !defined(DUK_EXP)
#define DUK_EXP              exp
#endif
#if !defined(DUK_LOG)
#define DUK_LOG              log
#endif
#if !defined(DUK_SQRT)
#define DUK_SQRT             sqrt
#endif

/* The functions below exist only in C99/C++11 or later and need a workaround
 * for platforms that don't include them.  MSVC isn't detected as C99, but
 * these functions also exist in MSVC 2013 and later so include a clause for
 * that too.  Android doesn't have log2; disable all of these for Android.
 */
#if (defined(DUK_F_C99) || defined(DUK_F_CPP11) || (defined(_MSC_VER) && (_MSC_VER >= 1800))) && \
    !defined(DUK_F_ANDROID) && !defined(DUK_F_MINT)
#if !defined(DUK_CBRT)
#define DUK_CBRT             cbrt
#endif
#if !defined(DUK_LOG2)
#define DUK_LOG2             log2
#endif
#if !defined(DUK_LOG10)
#define DUK_LOG10            log10
#endif
#if !defined(DUK_TRUNC)
#define DUK_TRUNC            trunc
#endif
#endif  /* DUK_F_C99 etc */

/* NetBSD 6.0 x86 (at least) has a few problems with pow() semantics,
 * see test-bug-netbsd-math-pow.js.  MinGW has similar (but different)
 * issues, see test-bug-mingw-math-issues.js.  Enable pow() workarounds
 * for these targets.
 */
#undef DUK_USE_POW_WORKAROUNDS
#if defined(DUK_F_NETBSD) || defined(DUK_F_MINGW)
#define DUK_USE_POW_WORKAROUNDS
#endif

/* Similar workarounds for atan2() semantics issues.  MinGW issues are
 * documented in test-bug-mingw-math-issues.js.
 */
#undef DUK_USE_ATAN2_WORKAROUNDS
#if defined(DUK_F_MINGW)
#define DUK_USE_ATAN2_WORKAROUNDS
#endif

/* Rely as little as possible on compiler behavior for NaN comparison,
 * signed zero handling, etc.  Currently never activated but may be needed
 * for broken compilers.
 */
#undef DUK_USE_PARANOID_MATH

/* There was a curious bug where test-bi-date-canceling.js would fail e.g.
 * on 64-bit Ubuntu, gcc-4.8.1, -m32, and no -std=c99.  Some date computations
 * using doubles would be optimized which then broke some corner case tests.
 * The problem goes away by adding 'volatile' to the datetime computations.
 * Not sure what the actual triggering conditions are, but using this on
 * non-C99 systems solves the known issues and has relatively little cost
 * on other platforms.
 */
#undef DUK_USE_PARANOID_DATE_COMPUTATION
#if !defined(DUK_F_C99)
#define DUK_USE_PARANOID_DATE_COMPUTATION
#endif

/*
 *  Byte order and double memory layout detection
 *
 *  Endianness detection is a major portability hassle because the macros
 *  and headers are not standardized.  There's even variance across UNIX
 *  platforms.  Even with "standard" headers, details like underscore count
 *  varies between platforms, e.g. both __BYTE_ORDER and _BYTE_ORDER are used
 *  (Crossbridge has a single underscore, for instance).
 *
 *  The checks below are structured with this in mind: several approaches are
 *  used, and at the end we check if any of them worked.  This allows generic
 *  approaches to be tried first, and platform/compiler specific hacks tried
 *  last.  As a last resort, the user can force a specific endianness, as it's
 *  not likely that automatic detection will work on the most exotic platforms.
 *
 *  Duktape supports little and big endian machines.  There's also support
 *  for a hybrid used by some ARM machines where integers are little endian
 *  but IEEE double values use a mixed order (12345678 -> 43218765).  This
 *  byte order for doubles is referred to as "mixed endian".
 */

/* GCC and Clang provide endianness defines as built-in predefines, with
 * leading and trailing double underscores (e.g. __BYTE_ORDER__).  See
 * output of "make gccpredefs" and "make clangpredefs".  Clang doesn't
 * seem to provide __FLOAT_WORD_ORDER__; assume not mixed endian for clang.
 * http://gcc.gnu.org/onlinedocs/cpp/Common-Predefined-Macros.html
 */
#if !defined(DUK_USE_BYTEORDER) && defined(__BYTE_ORDER__)
#if defined(__ORDER_LITTLE_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#if defined(__FLOAT_WORD_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && (__FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#define DUK_USE_BYTEORDER 1
#elif defined(__FLOAT_WORD_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__)
#define DUK_USE_BYTEORDER 2
#elif !defined(__FLOAT_WORD_ORDER__)
/* Float word order not known, assume not a hybrid. */
#define DUK_USE_BYTEORDER 1
#else
/* Byte order is little endian but cannot determine IEEE double word order. */
#endif  /* float word order */
#elif defined(__ORDER_BIG_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#if defined(__FLOAT_WORD_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__FLOAT_WORD_ORDER__ == __ORDER_BIG_ENDIAN__)
#define DUK_USE_BYTEORDER 3
#elif !defined(__FLOAT_WORD_ORDER__)
/* Float word order not known, assume not a hybrid. */
#define DUK_USE_BYTEORDER 3
#else
/* Byte order is big endian but cannot determine IEEE double word order. */
#endif  /* float word order */
#else
/* Cannot determine byte order; __ORDER_PDP_ENDIAN__ is related to 32-bit
 * integer ordering and is not relevant.
 */
#endif  /* integer byte order */
#endif  /* !defined(DUK_USE_BYTEORDER) && defined(__BYTE_ORDER__) */

/* More or less standard endianness predefines provided by header files.
 * The ARM hybrid case is detected by assuming that __FLOAT_WORD_ORDER
 * will be big endian, see: http://lists.mysql.com/internals/443.
 * On some platforms some defines may be present with an empty value which
 * causes comparisons to fail: https://github.com/svaarala/duktape/issues/453.
 */
#if !defined(DUK_USE_BYTEORDER)
#if defined(__BYTE_ORDER) && defined(__LITTLE_ENDIAN) && (__BYTE_ORDER == __LITTLE_ENDIAN) || \
    defined(_BYTE_ORDER) && defined(_LITTLE_ENDIAN) && (_BYTE_ORDER == _LITTLE_ENDIAN) || \
    defined(__LITTLE_ENDIAN__)
#if defined(__FLOAT_WORD_ORDER) && defined(__LITTLE_ENDIAN) && (__FLOAT_WORD_ORDER == __LITTLE_ENDIAN) || \
    defined(_FLOAT_WORD_ORDER) && defined(_LITTLE_ENDIAN) && (_FLOAT_WORD_ORDER == _LITTLE_ENDIAN)
#define DUK_USE_BYTEORDER 1
#elif defined(__FLOAT_WORD_ORDER) && defined(__BIG_ENDIAN) && (__FLOAT_WORD_ORDER == __BIG_ENDIAN) || \
      defined(_FLOAT_WORD_ORDER) && defined(_BIG_ENDIAN) && (_FLOAT_WORD_ORDER == _BIG_ENDIAN)
#define DUK_USE_BYTEORDER 2
#elif !defined(__FLOAT_WORD_ORDER) && !defined(_FLOAT_WORD_ORDER)
/* Float word order not known, assume not a hybrid. */
#define DUK_USE_BYTEORDER 1
#else
/* Byte order is little endian but cannot determine IEEE double word order. */
#endif  /* float word order */
#elif defined(__BYTE_ORDER) && defined(__BIG_ENDIAN) && (__BYTE_ORDER == __BIG_ENDIAN) || \
      defined(_BYTE_ORDER) && defined(_BIG_ENDIAN) && (_BYTE_ORDER == _BIG_ENDIAN) || \
      defined(__BIG_ENDIAN__)
#if defined(__FLOAT_WORD_ORDER) && defined(__BIG_ENDIAN) && (__FLOAT_WORD_ORDER == __BIG_ENDIAN) || \
    defined(_FLOAT_WORD_ORDER) && defined(_BIG_ENDIAN) && (_FLOAT_WORD_ORDER == _BIG_ENDIAN)
#define DUK_USE_BYTEORDER 3
#elif !defined(__FLOAT_WORD_ORDER) && !defined(_FLOAT_WORD_ORDER)
/* Float word order not known, assume not a hybrid. */
#define DUK_USE_BYTEORDER 3
#else
/* Byte order is big endian but cannot determine IEEE double word order. */
#endif  /* float word order */
#else
/* Cannot determine byte order. */
#endif  /* integer byte order */
#endif  /* !defined(DUK_USE_BYTEORDER) */

/* QNX gcc cross compiler seems to define e.g. __LITTLEENDIAN__ or __BIGENDIAN__:
 *  $ /opt/qnx650/host/linux/x86/usr/bin/i486-pc-nto-qnx6.5.0-gcc -dM -E - </dev/null | grep -ni endian
 *  67:#define __LITTLEENDIAN__ 1
 *  $ /opt/qnx650/host/linux/x86/usr/bin/mips-unknown-nto-qnx6.5.0-gcc -dM -E - </dev/null | grep -ni endian
 *  81:#define __BIGENDIAN__ 1
 *  $ /opt/qnx650/host/linux/x86/usr/bin/arm-unknown-nto-qnx6.5.0-gcc -dM -E - </dev/null | grep -ni endian
 *  70:#define __LITTLEENDIAN__ 1
 */
#if !defined(DUK_USE_BYTEORDER)
#if defined(__LITTLEENDIAN__)
#define DUK_USE_BYTEORDER 1
#elif defined(__BIGENDIAN__)
#define DUK_USE_BYTEORDER 3
#endif
#endif

/*
 *  Alignment requirement and support for unaligned accesses
 *
 *  Assume unaligned accesses are not supported unless specifically allowed
 *  in the target platform.  Some platforms may support unaligned accesses
 *  but alignment to 4 or 8 may still be desirable.  Note that unaligned
 *  accesses (and even pointers) relative to natural alignment (regardless
 *  of target alignment) are technically undefined behavior and thus
 *  compiler/architecture specific.
 */

/* If not forced, use safe default for alignment. */
#if !defined(DUK_USE_ALIGN_BY)
#define DUK_USE_ALIGN_BY 8
#endif

/* Compiler specific hackery needed to force struct size to match alignment,
 * see e.g. duk_hbuffer.h.
 *
 * http://stackoverflow.com/questions/11130109/c-struct-size-alignment
 * http://stackoverflow.com/questions/10951039/specifying-64-bit-alignment
 */
#if !(defined(DUK_USE_PACK_MSVC_PRAGMA) || defined(DUK_USE_PACK_GCC_ATTR) || \
      defined(DUK_USE_PACK_CLANG_ATTR) || defined(DUK_USE_PACK_DUMMY_MEMBER))
#define DUK_USE_PACK_DUMMY_MEMBER
#endif

#if !defined(DUK_U64_CONSTANT)
#define DUK_U64_CONSTANT(x) x##ULL
#endif
#if !defined(DUK_I64_CONSTANT)
#define DUK_I64_CONSTANT(x) x##LL
#endif

#if !defined(DUK_VA_COPY)
/* We need va_copy() which is defined in C99 / C++11, so an awkward
 * replacement is needed for pre-C99 / pre-C++11 environments.  This
 * will quite likely need portability hacks for some non-C99
 * environments.
 */
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
/* C99 / C++11 and above: rely on va_copy() which is required.
 * Omit parenthesis on macro right side on purpose to minimize differences
 * to direct use.
 */
#define DUK_VA_COPY(dest,src) va_copy(dest,src)
#else
/* Pre-C99: va_list type is implementation dependent.  This replacement
 * assumes it is a plain value so that a simple assignment will work.
 * This is not the case on all platforms (it may be a single-array element,
 * for instance).
 */
#define DUK_VA_COPY(dest,src) do { (dest) = (src); } while (0)
#endif
#endif

#if !defined(DUK_MACRO_STRINGIFY)
/* Macro hackery to convert e.g. __LINE__ to a string without formatting,
 * see: http://stackoverflow.com/questions/240353/convert-a-preprocessor-token-to-a-string
 */
#define DUK_MACRO_STRINGIFY_HELPER(x)  #x
#define DUK_MACRO_STRINGIFY(x)  DUK_MACRO_STRINGIFY_HELPER(x)
#endif

#if !defined(DUK_CAUSE_SEGFAULT)
/* This can be used for testing; valgrind will then indicate the C call stack
 * leading to the call site.
 */
#define DUK_CAUSE_SEGFAULT()  do { *((volatile duk_uint32_t *) NULL) = (duk_uint32_t) 0xdeadbeefUL; } while (0)
#endif

#if !defined(DUK_UNREF)
/* Macro for suppressing warnings for potentially unreferenced variables.
 * The variables can be actually unreferenced or unreferenced in some
 * specific cases only; for instance, if a variable is only debug printed,
 * it is unreferenced when debug printing is disabled.  May cause warnings
 * for volatile arguments.
 */
#define DUK_UNREF(x)  do { (void) (x); } while (0)
#endif

/* Fillin for DUK_NORETURN; DUK_WO_NORETURN() is used to insert dummy
 * dummy statements after noreturn calls to silence harmless compiler
 * warnings, e.g.:
 *
 *   DUK_ERROR_TYPE(thr, "aiee");
 *   DUK_WO_NORETURN(return 0;);
 *
 * Statements inside DUK_WO_NORETURN() must NEVER be actually reachable,
 * and they're only included to satisfy the compiler.
 */
#if defined(DUK_NORETURN)
#define DUK_WO_NORETURN(stmt) do { } while (0)
#else
#define DUK_NORETURN(decl)  decl
#define DUK_WO_NORETURN(stmt) do { stmt } while (0)
#endif

#if defined(DUK_UNREACHABLE)
#define DUK_WO_UNREACHABLE(stmt) do { } while (0)
#else
/* Don't know how to declare unreachable point, so don't do it; this
 * may cause some spurious compilation warnings (e.g. "variable used
 * uninitialized").
 */
#define DUK_UNREACHABLE()  do { } while (0)
#define DUK_WO_UNREACHABLE(stmt) do { stmt } while (0)
#endif

#if !defined(DUK_LOSE_CONST)
/* Convert any input pointer into a "void *", losing a const qualifier.
 * This is not fully portable because casting through duk_uintptr_t may
 * not work on all architectures (e.g. those with long, segmented pointers).
 */
#define DUK_LOSE_CONST(src) ((void *) (duk_uintptr_t) (src))
#endif

#if !defined(DUK_LIKELY)
#define DUK_LIKELY(x)    (x)
#endif
#if !defined(DUK_UNLIKELY)
#define DUK_UNLIKELY(x)  (x)
#endif
#if !defined(DUK_UNPREDICTABLE)
#define DUK_UNPREDICTABLE(x)  (x)
#endif

#if !defined(DUK_NOINLINE)
#define DUK_NOINLINE       /*nop*/
#endif
#if !defined(DUK_INLINE)
#define DUK_INLINE         /*nop*/
#endif
#if !defined(DUK_ALWAYS_INLINE)
#define DUK_ALWAYS_INLINE  /*nop*/
#endif

#if !defined(DUK_HOT)
#define DUK_HOT            /*nop*/
#endif
#if !defined(DUK_COLD)
#define DUK_COLD           /*nop*/
#endif

#if !defined(DUK_EXTERNAL_DECL)
#define DUK_EXTERNAL_DECL  extern
#endif
#if !defined(DUK_EXTERNAL)
#define DUK_EXTERNAL       /*empty*/
#endif
#if !defined(DUK_INTERNAL_DECL)
#if defined(DUK_SINGLE_FILE)
#define DUK_INTERNAL_DECL  static
#else
#define DUK_INTERNAL_DECL  extern
#endif
#endif
#if !defined(DUK_INTERNAL)
#if defined(DUK_SINGLE_FILE)
#define DUK_INTERNAL       static
#else
#define DUK_INTERNAL       /*empty*/
#endif
#endif
#if !defined(DUK_LOCAL_DECL)
#define DUK_LOCAL_DECL     static
#endif
#if !defined(DUK_LOCAL)
#define DUK_LOCAL          static
#endif

#if !defined(DUK_FILE_MACRO)
#define DUK_FILE_MACRO  __FILE__
#endif
#if !defined(DUK_LINE_MACRO)
#define DUK_LINE_MACRO  __LINE__
#endif
#if !defined(DUK_FUNC_MACRO)
#if defined(DUK_F_C99) || defined(DUK_F_CPP11)
#define DUK_FUNC_MACRO  __func__
#elif defined(__FUNCTION__)
#define DUK_FUNC_MACRO  __FUNCTION__
#else
#define DUK_FUNC_MACRO  "unknown"
#endif
#endif

#if defined(DUK_F_HAVE_64BIT)
#if !defined(DUK_BSWAP64)
#define DUK_BSWAP64(x) \
	((((duk_uint64_t) (x)) >> 56U) | \
	 ((((duk_uint64_t) (x)) >> 40U) & DUK_U64_CONSTANT(0xff00)) | \
	 ((((duk_uint64_t) (x)) >> 24U) & DUK_U64_CONSTANT(0xff0000)) | \
	 ((((duk_uint64_t) (x)) >> 8U) & DUK_U64_CONSTANT(0xff000000)) | \
	 ((((duk_uint64_t) (x)) << 8U) & DUK_U64_CONSTANT(0xff00000000)) | \
	 ((((duk_uint64_t) (x)) << 24U) & DUK_U64_CONSTANT(0xff0000000000)) | \
	 ((((duk_uint64_t) (x)) << 40U) & DUK_U64_CONSTANT(0xff000000000000)) | \
	 (((duk_uint64_t) (x)) << 56U))
#endif
#endif
#if !defined(DUK_BSWAP32)
#define DUK_BSWAP32(x) \
	((((duk_uint32_t) (x)) >> 24U) | \
	 ((((duk_uint32_t) (x)) >> 8U) & 0xff00UL) | \
	 ((((duk_uint32_t) (x)) << 8U) & 0xff0000UL) | \
	 (((duk_uint32_t) (x)) << 24U))
#endif
#if !defined(DUK_BSWAP16)
#define DUK_BSWAP16(x) \
	((duk_uint16_t) (x) >> 8U) | \
	((duk_uint16_t) (x) << 8U)
#endif

/* DUK_USE_VARIADIC_MACROS: required from compilers, so no fill-in. */
/* DUK_USE_UNION_INITIALIZERS: required from compilers, so no fill-in. */

#if !(defined(DUK_USE_FLEX_C99) || defined(DUK_USE_FLEX_ZEROSIZE) || defined(DUK_USE_FLEX_ONESIZE))
#if defined(DUK_F_C99)
#define DUK_USE_FLEX_C99
#else
#define DUK_USE_FLEX_ZEROSIZE  /* Not standard but common enough */
#endif
#endif

#if !(defined(DUK_USE_PACK_GCC_ATTR) || defined(DUK_USE_PACK_CLANG_ATTR) || \
      defined(DUK_USE_PACK_MSVC_PRAGMA) || defined(DUK_USE_PACK_DUMMY_MEMBER))
#define DUK_USE_PACK_DUMMY_MEMBER
#endif

#if 0  /* not defined by default */
#undef DUK_USE_GCC_PRAGMAS
#endif

/* Workaround for GH-323: avoid inlining control when compiling from
 * multiple sources, as it causes compiler portability trouble.
 */
#if !defined(DUK_SINGLE_FILE)
#undef DUK_NOINLINE
#undef DUK_INLINE
#undef DUK_ALWAYS_INLINE
#define DUK_NOINLINE       /*nop*/
#define DUK_INLINE         /*nop*/
#define DUK_ALWAYS_INLINE  /*nop*/
#endif

/*
 *  Check whether or not a packed duk_tval representation is possible.
 *  What's basically required is that pointers are 32-bit values
 *  (sizeof(void *) == 4).  Best effort check, not always accurate.
 *  If guess goes wrong, crashes may result; self tests also verify
 *  the guess.
 */

/* Explicit marker needed; may be 'defined', 'undefined, 'or 'not provided'. */
#if !defined(DUK_F_PACKED_TVAL_PROVIDED)
#undef DUK_F_PACKED_TVAL_POSSIBLE

/* Strict C99 case: DUK_UINTPTR_MAX (= UINTPTR_MAX) should be very reliable */
#if !defined(DUK_F_PACKED_TVAL_POSSIBLE) && defined(DUK_UINTPTR_MAX)
#if (DUK_UINTPTR_MAX <= 0xffffffffUL)
#define DUK_F_PACKED_TVAL_POSSIBLE
#endif
#endif

/* Non-C99 case, still relying on DUK_UINTPTR_MAX, as long as it is not a computed value */
#if !defined(DUK_F_PACKED_TVAL_POSSIBLE) && defined(DUK_UINTPTR_MAX) && !defined(DUK_UINTPTR_MAX_COMPUTED)
#if (DUK_UINTPTR_MAX <= 0xffffffffUL)
#define DUK_F_PACKED_TVAL_POSSIBLE
#endif
#endif

/* DUK_SIZE_MAX (= SIZE_MAX) is often reliable */
#if !defined(DUK_F_PACKED_TVAL_POSSIBLE) && defined(DUK_SIZE_MAX) && !defined(DUK_SIZE_MAX_COMPUTED)
#if (DUK_SIZE_MAX <= 0xffffffffUL)
#define DUK_F_PACKED_TVAL_POSSIBLE
#endif
#endif

#undef DUK_USE_PACKED_TVAL
#if defined(DUK_F_PACKED_TVAL_POSSIBLE)
#define DUK_USE_PACKED_TVAL
#endif
#undef DUK_F_PACKED_TVAL_POSSIBLE

#endif  /* DUK_F_PACKED_TVAL_PROVIDED */

/* No memory layout defines for duk_hobject at present. */

/* GCC/clang inaccurate math would break compliance and probably duk_tval,
 * so refuse to compile.  Relax this if -ffast-math is tested to work.
 */
#if defined(__FAST_MATH__)
#error __FAST_MATH__ defined, refusing to compile
#endif

/*
 *  Config options
 */

/* Forced options */
/* Default options */
#undef DUK_USE_ALLOW_UNDEFINED_BEHAVIOR
#define DUK_USE_ARRAY_BUILTIN
#define DUK_USE_ARRAY_FASTPATH
#define DUK_USE_ARRAY_PROP_FASTPATH
#undef DUK_USE_ASSERTIONS
#define DUK_USE_AUGMENT_ERROR_CREATE
#define DUK_USE_AUGMENT_ERROR_THROW
#define DUK_USE_AVOID_PLATFORM_FUNCPTRS
#define DUK_USE_BASE64_FASTPATH
#define DUK_USE_BASE64_SUPPORT
#define DUK_USE_BOOLEAN_BUILTIN
#define DUK_USE_BUFFEROBJECT_SUPPORT
#undef DUK_USE_BUFLEN16
#define DUK_USE_BYTECODE_DUMP_SUPPORT
#define DUK_USE_CACHE_ACTIVATION
#define DUK_USE_CACHE_CATCHER
#define DUK_USE_CALLSTACK_LIMIT 10000
#define DUK_USE_CBOR_BUILTIN
#define DUK_USE_CBOR_DEC_RECLIMIT 1000
#define DUK_USE_CBOR_ENC_RECLIMIT 1000
#define DUK_USE_CBOR_SUPPORT
#define DUK_USE_COMPILER_RECLIMIT 2500
#define DUK_USE_COROUTINE_SUPPORT
#undef DUK_USE_CPP_EXCEPTIONS
#undef DUK_USE_DATAPTR16
#undef DUK_USE_DATAPTR_DEC16
#undef DUK_USE_DATAPTR_ENC16
#define DUK_USE_DATE_BUILTIN
#undef DUK_USE_DATE_FORMAT_STRING
#undef DUK_USE_DATE_GET_LOCAL_TZOFFSET
#undef DUK_USE_DATE_GET_NOW
#undef DUK_USE_DATE_PARSE_STRING
#undef DUK_USE_DATE_PRS_GETDATE
#undef DUK_USE_DEBUG
#undef DUK_USE_DEBUGGER_DUMPHEAP
#undef DUK_USE_DEBUGGER_INSPECT
#undef DUK_USE_DEBUGGER_PAUSE_UNCAUGHT
#undef DUK_USE_DEBUGGER_SUPPORT
#define DUK_USE_DEBUGGER_THROW_NOTIFY
#undef DUK_USE_DEBUGGER_TRANSPORT_TORTURE
#define DUK_USE_DEBUG_BUFSIZE 65536L
#define DUK_USE_DEBUG_LEVEL 0
#undef DUK_USE_DEBUG_WRITE
#define DUK_USE_DOUBLE_LINKED_HEAP
#define DUK_USE_DUKTAPE_BUILTIN
#define DUK_USE_ENCODING_BUILTINS
#define DUK_USE_ERRCREATE
#define DUK_USE_ERRTHROW
#define DUK_USE_ES6
#define DUK_USE_ES6_OBJECT_PROTO_PROPERTY
#define DUK_USE_ES6_OBJECT_SETPROTOTYPEOF
#define DUK_USE_ES6_PROXY
#define DUK_USE_ES6_REGEXP_SYNTAX
#define DUK_USE_ES6_UNICODE_ESCAPE
#define DUK_USE_ES7
#define DUK_USE_ES7_EXP_OPERATOR
#define DUK_USE_ES8
#define DUK_USE_ES9
#define DUK_USE_ESBC_LIMITS
#define DUK_USE_ESBC_MAX_BYTES 2147418112L
#define DUK_USE_ESBC_MAX_LINENUMBER 2147418112L
#undef DUK_USE_EXEC_FUN_LOCAL
#undef DUK_USE_EXEC_INDIRECT_BOUND_CHECK
#undef DUK_USE_EXEC_PREFER_SIZE
#define DUK_USE_EXEC_REGCONST_OPTIMIZE
#undef DUK_USE_EXEC_TIMEOUT_CHECK
#undef DUK_USE_EXPLICIT_NULL_INIT
#undef DUK_USE_EXTSTR_FREE
#undef DUK_USE_EXTSTR_INTERN_CHECK
#undef DUK_USE_FASTINT
#define DUK_USE_FAST_REFCOUNT_DEFAULT
#undef DUK_USE_FATAL_HANDLER
#define DUK_USE_FATAL_MAXLEN 128
#define DUK_USE_FINALIZER_SUPPORT
#undef DUK_USE_FINALIZER_TORTURE
#undef DUK_USE_FUNCPTR16
#undef DUK_USE_FUNCPTR_DEC16
#undef DUK_USE_FUNCPTR_ENC16
#define DUK_USE_FUNCTION_BUILTIN
#define DUK_USE_FUNC_FILENAME_PROPERTY
#define DUK_USE_FUNC_NAME_PROPERTY
#undef DUK_USE_FUZZILLI
#undef DUK_USE_GC_TORTURE
#undef DUK_USE_GET_MONOTONIC_TIME
#undef DUK_USE_GET_RANDOM_DOUBLE
#define DUK_USE_GLOBAL_BINDING
#define DUK_USE_GLOBAL_BUILTIN
#undef DUK_USE_HEAPPTR16
#undef DUK_USE_HEAPPTR_DEC16
#undef DUK_USE_HEAPPTR_ENC16
#define DUK_USE_HEX_FASTPATH
#define DUK_USE_HEX_SUPPORT
#define DUK_USE_HOBJECT_ARRAY_ABANDON_LIMIT 2
#define DUK_USE_HOBJECT_ARRAY_ABANDON_MINSIZE 257
#define DUK_USE_HOBJECT_ARRAY_FAST_RESIZE_LIMIT 9
#define DUK_USE_HOBJECT_ARRAY_MINGROW_ADD 16
#define DUK_USE_HOBJECT_ARRAY_MINGROW_DIVISOR 8
#define DUK_USE_HOBJECT_ENTRY_MINGROW_ADD 16
#define DUK_USE_HOBJECT_ENTRY_MINGROW_DIVISOR 8
#define DUK_USE_HOBJECT_HASH_PART
#define DUK_USE_HOBJECT_HASH_PROP_LIMIT 8
#define DUK_USE_HSTRING_ARRIDX
#define DUK_USE_HSTRING_CLEN
#undef DUK_USE_HSTRING_EXTDATA
#define DUK_USE_HTML_COMMENTS
#define DUK_USE_IDCHAR_FASTPATH
#undef DUK_USE_INJECT_HEAP_ALLOC_ERROR
#undef DUK_USE_INTERRUPT_COUNTER
#undef DUK_USE_INTERRUPT_DEBUG_FIXUP
#define DUK_USE_JC
#define DUK_USE_JSON_BUILTIN
#define DUK_USE_JSON_DECNUMBER_FASTPATH
#define DUK_USE_JSON_DECSTRING_FASTPATH
#define DUK_USE_JSON_DEC_RECLIMIT 1000
#define DUK_USE_JSON_EATWHITE_FASTPATH
#define DUK_USE_JSON_ENC_RECLIMIT 1000
#define DUK_USE_JSON_QUOTESTRING_FASTPATH
#undef DUK_USE_JSON_STRINGIFY_FASTPATH
#define DUK_USE_JSON_SUPPORT
#define DUK_USE_JX
#define DUK_USE_LEXER_SLIDING_WINDOW
#undef DUK_USE_LIGHTFUNC_BUILTINS
#define DUK_USE_LITCACHE_SIZE 256
#define DUK_USE_MARK_AND_SWEEP_RECLIMIT 256
#define DUK_USE_MATH_BUILTIN
#define DUK_USE_NATIVE_CALL_RECLIMIT 1000
#undef DUK_USE_NATIVE_STACK_CHECK
#undef DUK_USE_NONSTD_FUNC_SOURCE_PROPERTY
#define DUK_USE_NONSTD_FUNC_STMT
#define DUK_USE_NONSTD_GETTER_KEY_ARGUMENT
#define DUK_USE_NONSTD_JSON_ESC_U2028_U2029
#define DUK_USE_NONSTD_SETTER_KEY_ARGUMENT
#define DUK_USE_NUMBER_BUILTIN
#define DUK_USE_OBJECT_BUILTIN
#undef DUK_USE_OBJSIZES16
#undef DUK_USE_PARANOID_ERRORS
#define DUK_USE_PC2LINE
#define DUK_USE_PERFORMANCE_BUILTIN
#undef DUK_USE_PREFER_SIZE
#undef DUK_USE_PROMISE_BUILTIN
#define DUK_USE_PROVIDE_DEFAULT_ALLOC_FUNCTIONS
#define DUK_USE_PROXY_POLICY
#undef DUK_USE_REFCOUNT16
#define DUK_USE_REFCOUNT32
#define DUK_USE_REFERENCE_COUNTING
#define DUK_USE_REFLECT_BUILTIN
#define DUK_USE_REGEXP_CANON_BITMAP
#undef DUK_USE_REGEXP_CANON_WORKAROUND
#define DUK_USE_REGEXP_COMPILER_RECLIMIT 10000
#define DUK_USE_REGEXP_EXECUTOR_RECLIMIT 10000
#define DUK_USE_REGEXP_SUPPORT
#undef DUK_USE_ROM_GLOBAL_CLONE
#undef DUK_USE_ROM_GLOBAL_INHERIT
#undef DUK_USE_ROM_OBJECTS
#define DUK_USE_ROM_PTRCOMP_FIRST 63488L
#undef DUK_USE_ROM_STRINGS
#define DUK_USE_SECTION_B
#undef DUK_USE_SELF_TESTS
#define DUK_USE_SHEBANG_COMMENTS
#undef DUK_USE_SHUFFLE_TORTURE
#define DUK_USE_SOURCE_NONBMP
#undef DUK_USE_STRHASH16
#undef DUK_USE_STRHASH_DENSE
#define DUK_USE_STRHASH_SKIP_SHIFT 5
#define DUK_USE_STRICT_DECL
#undef DUK_USE_STRICT_UTF8_SOURCE
#define DUK_USE_STRING_BUILTIN
#undef DUK_USE_STRLEN16
#define DUK_USE_STRTAB_GROW_LIMIT 17
#define DUK_USE_STRTAB_MAXSIZE 268435456L
#define DUK_USE_STRTAB_MINSIZE 1024
#undef DUK_USE_STRTAB_PTRCOMP
#define DUK_USE_STRTAB_RESIZE_CHECK_MASK 255
#define DUK_USE_STRTAB_SHRINK_LIMIT 6
#undef DUK_USE_STRTAB_TORTURE
#define DUK_USE_SYMBOL_BUILTIN
#define DUK_USE_TAILCALL
#define DUK_USE_TARGET_INFO "unknown"
#define DUK_USE_TRACEBACKS
#define DUK_USE_TRACEBACK_DEPTH 10
#define DUK_USE_VALSTACK_GROW_SHIFT 2
#define DUK_USE_VALSTACK_LIMIT 1000000L
#define DUK_USE_VALSTACK_SHRINK_CHECK_SHIFT 2
#define DUK_USE_VALSTACK_SHRINK_SLACK_SHIFT 4
#undef DUK_USE_VALSTACK_UNSAFE
#define DUK_USE_VERBOSE_ERRORS
#define DUK_USE_VERBOSE_EXECUTOR_ERRORS
#define DUK_USE_VOLUNTARY_GC
#define DUK_USE_ZERO_BUFFER_DATA

/*
 *  Fixups
 */

/*
 *  You may add overriding #define/#undef directives below for
 *  customization.  You of course cannot un-#include or un-typedef
 *  anything; these require direct changes above.
 */

/* __OVERRIDE_DEFINES__ */

/*
 *  Conditional includes
 */

#if defined(DUK_F_CPP) && defined(DUK_USE_CPP_EXCEPTIONS)
#include <exception>  /* std::exception */
#include <stdexcept>  /* std::runtime_error */
#endif

/*
 *  Date provider selection
 *
 *  User may define DUK_USE_DATE_GET_NOW() etc directly, in which case we'll
 *  rely on an external provider.  If this is not done, revert to previous
 *  behavior and use Unix/Windows built-in provider.
 */

#if defined(DUK_COMPILING_DUKTAPE)

#if defined(DUK_USE_DATE_GET_NOW)
/* External provider already defined. */
#elif defined(DUK_USE_DATE_NOW_GETTIMEOFDAY)
#define DUK_USE_DATE_GET_NOW(ctx)            duk_bi_date_get_now_gettimeofday()
#elif defined(DUK_USE_DATE_NOW_TIME)
#define DUK_USE_DATE_GET_NOW(ctx)            duk_bi_date_get_now_time()
#elif defined(DUK_USE_DATE_NOW_WINDOWS)
#define DUK_USE_DATE_GET_NOW(ctx)            duk_bi_date_get_now_windows()
#elif defined(DUK_USE_DATE_NOW_WINDOWS_SUBMS)
#define DUK_USE_DATE_GET_NOW(ctx)            duk_bi_date_get_now_windows_subms()
#else
#error no provider for DUK_USE_DATE_GET_NOW()
#endif

#if defined(DUK_USE_DATE_GET_LOCAL_TZOFFSET)
/* External provider already defined. */
#elif defined(DUK_USE_DATE_TZO_GMTIME_R) || defined(DUK_USE_DATE_TZO_GMTIME_S) || defined(DUK_USE_DATE_TZO_GMTIME)
#define DUK_USE_DATE_GET_LOCAL_TZOFFSET(d)   duk_bi_date_get_local_tzoffset_gmtime((d))
#elif defined(DUK_USE_DATE_TZO_WINDOWS)
#define DUK_USE_DATE_GET_LOCAL_TZOFFSET(d)   duk_bi_date_get_local_tzoffset_windows((d))
#elif defined(DUK_USE_DATE_TZO_WINDOWS_NO_DST)
#define DUK_USE_DATE_GET_LOCAL_TZOFFSET(d)   duk_bi_date_get_local_tzoffset_windows_no_dst((d))
#else
#error no provider for DUK_USE_DATE_GET_LOCAL_TZOFFSET()
#endif

#if defined(DUK_USE_DATE_PARSE_STRING)
/* External provider already defined. */
#elif defined(DUK_USE_DATE_PRS_STRPTIME)
#define DUK_USE_DATE_PARSE_STRING(ctx,str)   duk_bi_date_parse_string_strptime((ctx), (str))
#elif defined(DUK_USE_DATE_PRS_GETDATE)
#define DUK_USE_DATE_PARSE_STRING(ctx,str)   duk_bi_date_parse_string_getdate((ctx), (str))
#else
/* No provider for DUK_USE_DATE_PARSE_STRING(), fall back to ISO 8601 only. */
#endif

#if defined(DUK_USE_DATE_FORMAT_STRING)
/* External provider already defined. */
#elif defined(DUK_USE_DATE_FMT_STRFTIME)
#define DUK_USE_DATE_FORMAT_STRING(ctx,parts,tzoffset,flags) \
	duk_bi_date_format_parts_strftime((ctx), (parts), (tzoffset), (flags))
#else
/* No provider for DUK_USE_DATE_FORMAT_STRING(), fall back to ISO 8601 only. */
#endif

#if defined(DUK_USE_GET_MONOTONIC_TIME)
/* External provider already defined. */
#elif defined(DUK_USE_GET_MONOTONIC_TIME_CLOCK_GETTIME)
#define DUK_USE_GET_MONOTONIC_TIME(ctx)  duk_bi_date_get_monotonic_time_clock_gettime()
#elif defined(DUK_USE_GET_MONOTONIC_TIME_WINDOWS_QPC)
#define DUK_USE_GET_MONOTONIC_TIME(ctx)  duk_bi_date_get_monotonic_time_windows_qpc()
#else
/* No provider for DUK_USE_GET_MONOTONIC_TIME(), fall back to DUK_USE_DATE_GET_NOW(). */
#endif

#endif  /* DUK_COMPILING_DUKTAPE */

/*
 *  Convert DUK_USE_BYTEORDER, from whatever source, into currently used
 *  internal defines.  If detection failed, #error out.
 */

#if defined(DUK_USE_BYTEORDER)
#if (DUK_USE_BYTEORDER == 1)
#define DUK_USE_INTEGER_LE
#define DUK_USE_DOUBLE_LE
#elif (DUK_USE_BYTEORDER == 2)
#define DUK_USE_INTEGER_LE  /* integer endianness is little on purpose */
#define DUK_USE_DOUBLE_ME
#elif (DUK_USE_BYTEORDER == 3)
#define DUK_USE_INTEGER_BE
#define DUK_USE_DOUBLE_BE
#else
#error unsupported: byte order invalid
#endif  /* byte order */
#else
#error unsupported: byte order detection failed
#endif  /* defined(DUK_USE_BYTEORDER) */

#endif  /* DUK_CONFIG_H_INCLUDED */
