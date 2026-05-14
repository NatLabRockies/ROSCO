/*
 * pharlap_compat.c — Win32 API compatibility stubs for Phar Lap ETS
 *
 * Phar Lap ETS implements only a subset of the Win32 API.  MinGW's
 * statically-linked C++ runtime (libstdc++, libgcc, winpthreads) imports
 * many functions that Phar Lap's KERNEL32.dll doesn't provide.
 *
 * This file provides local stub implementations so that these symbols
 * are resolved from OUR object file instead of the import library.
 * For each function we provide:
 *   1. A stdcall function body  (resolves _FuncName@N)
 *   2. An __imp__ data pointer  (resolves __imp__FuncName@N)
 *
 * With both symbols satisfied locally, the linker never includes the
 * import-library archive member, keeping the function OUT of the PE
 * import table entirely.  Phar Lap never needs to provide them.
 *
 * All stubs assume single-threaded operation (safe for ROSCO on PXI).
 */

#ifdef ROSCO_PHARLAP

/* ---- minimal Win32 typedefs (avoid pulling in windows.h) ---- */
typedef unsigned long  DWORD;
typedef unsigned short WORD;
typedef int            BOOL;
typedef void*          HANDLE;
typedef void*          LPVOID;
typedef const char*    LPCSTR;
typedef const void*    LPCWSTR;
typedef unsigned long  ULONG_PTR;
typedef unsigned long  SIZE_T;
typedef long           LONG;

#define STDCALL __attribute__((stdcall))
#define NULL_HANDLE ((HANDLE)0)
#define PSEUDO_HANDLE ((HANDLE)(ULONG_PTR)-1)
#define TRUE  1
#define FALSE 0
#define WAIT_OBJECT_0 0

typedef struct {
    DWORD  dwLowDateTime;
    DWORD  dwHighDateTime;
} FILETIME;

typedef struct {
    DWORD  nLength;
    LPVOID lpSecurityDescriptor;
    BOOL   bInheritHandle;
} SECURITY_ATTRIBUTES;

/* ================================================================
 * KERNEL32.dll stubs
 * ================================================================ */

BOOL STDCALL CloseHandle(HANDLE h)
{ (void)h; return TRUE; }

HANDLE STDCALL CreateEventA(SECURITY_ATTRIBUTES* sa, BOOL mr, BOOL is, LPCSTR n)
{ (void)sa; (void)mr; (void)is; (void)n; return PSEUDO_HANDLE; }

HANDLE STDCALL CreateSemaphoreA(SECURITY_ATTRIBUTES* sa, LONG init, LONG max, LPCSTR n)
{ (void)sa; (void)init; (void)max; (void)n; return PSEUDO_HANDLE; }

BOOL STDCALL DuplicateHandle(HANDLE sp, HANDLE sh, HANDLE tp, HANDLE* th,
                             DWORD da, BOOL ih, DWORD opt)
{ (void)sp; (void)sh; (void)tp; (void)da; (void)ih; (void)opt;
  if (th) *th = PSEUDO_HANDLE; return TRUE; }

HANDLE STDCALL GetCurrentProcess(void)
{ return PSEUDO_HANDLE; }

DWORD STDCALL GetCurrentProcessId(void)
{ return 1; }

HANDLE STDCALL GetCurrentThread(void)
{ return (HANDLE)(ULONG_PTR)-2; }

DWORD STDCALL GetCurrentThreadId(void)
{ return 1; }

BOOL STDCALL GetHandleInformation(HANDLE h, DWORD* flags)
{ (void)h; if (flags) *flags = 0; return TRUE; }

HANDLE STDCALL GetModuleHandleW(LPCWSTR name)
{ (void)name; return NULL_HANDLE; }

BOOL STDCALL GetProcessAffinityMask(HANDLE p, ULONG_PTR* pm, ULONG_PTR* sm)
{ (void)p; if (pm) *pm = 1; if (sm) *sm = 1; return TRUE; }

void STDCALL GetSystemTimeAsFileTime(FILETIME* ft)
{ if (ft) { ft->dwLowDateTime = 0; ft->dwHighDateTime = 0; } }

BOOL STDCALL GetThreadContext(HANDLE t, LPVOID ctx)
{ (void)t; (void)ctx; return FALSE; }

int STDCALL GetThreadPriority(HANDLE t)
{ (void)t; return 0; }

DWORD STDCALL GetTickCount(void)
{ return 0; }

BOOL STDCALL IsDBCSLeadByteEx(unsigned int cp, unsigned char ch)
{ (void)cp; (void)ch; return FALSE; }

BOOL STDCALL IsDebuggerPresent(void)
{ return FALSE; }

int STDCALL MultiByteToWideChar(unsigned int cp, DWORD flags, LPCSTR mb, int cbmb,
                                void* wc, int cchwc)
{ (void)cp; (void)flags; (void)mb; (void)cbmb; (void)wc; (void)cchwc; return 0; }

HANDLE STDCALL OpenProcess(DWORD da, BOOL ih, DWORD pid)
{ (void)da; (void)ih; (void)pid; return NULL_HANDLE; }

void STDCALL OutputDebugStringA(LPCSTR s)
{ (void)s; }

BOOL STDCALL QueryPerformanceCounter(void* ctr)
{ if (ctr) { ((DWORD*)ctr)[0] = 0; ((DWORD*)ctr)[1] = 0; } return FALSE; }

BOOL STDCALL QueryPerformanceFrequency(void* freq)
{ if (freq) { ((DWORD*)freq)[0] = 0; ((DWORD*)freq)[1] = 0; } return FALSE; }

void STDCALL RaiseException(DWORD code, DWORD flags, DWORD nargs, const ULONG_PTR* args)
{ (void)code; (void)flags; (void)nargs; (void)args; }

BOOL STDCALL ReleaseSemaphore(HANDLE sem, LONG count, LONG* prev)
{ (void)sem; (void)count; if (prev) *prev = 0; return TRUE; }

BOOL STDCALL ResetEvent(HANDLE ev)
{ (void)ev; return TRUE; }

DWORD STDCALL ResumeThread(HANDLE t)
{ (void)t; return 0; }

BOOL STDCALL SetEvent(HANDLE ev)
{ (void)ev; return TRUE; }

static DWORD g_last_error = 0;

void STDCALL SetLastError(DWORD err)
{ g_last_error = err; }

BOOL STDCALL SetProcessAffinityMask(HANDLE p, ULONG_PTR mask)
{ (void)p; (void)mask; return TRUE; }

BOOL STDCALL SetThreadContext(HANDLE t, const void* ctx)
{ (void)t; (void)ctx; return FALSE; }

BOOL STDCALL SetThreadPriority(HANDLE t, int prio)
{ (void)t; (void)prio; return TRUE; }

DWORD STDCALL SuspendThread(HANDLE t)
{ (void)t; return 0; }

#define MAX_TLS_SLOTS 64
static LPVOID tls_values[MAX_TLS_SLOTS];
static DWORD  tls_next = 0;

DWORD STDCALL TlsAlloc(void)
{ if (tls_next < MAX_TLS_SLOTS) return tls_next++; return 0xFFFFFFFF; }

BOOL STDCALL TlsSetValue(DWORD idx, LPVOID val)
{ if (idx < MAX_TLS_SLOTS) { tls_values[idx] = val; return TRUE; } return FALSE; }

BOOL STDCALL TryEnterCriticalSection(void* cs)
{ (void)cs; return TRUE; }

DWORD STDCALL WaitForMultipleObjects(DWORD n, const HANDLE* h, BOOL all, DWORD ms)
{ (void)n; (void)h; (void)all; (void)ms; return WAIT_OBJECT_0; }

DWORD STDCALL WaitForSingleObject(HANDLE h, DWORD ms)
{ (void)h; (void)ms; return WAIT_OBJECT_0; }

int STDCALL WideCharToMultiByte(unsigned int cp, DWORD flags, LPCWSTR wc, int cchwc,
                                char* mb, int cbmb, LPCSTR defch, BOOL* used)
{ (void)cp; (void)flags; (void)wc; (void)cchwc; (void)mb; (void)cbmb;
  (void)defch; (void)used; return 0; }

/* --- additional KERNEL32 stubs needed by libdiscon --- */

DWORD STDCALL FormatMessageA(DWORD flags, const void* src, DWORD msgid,
                             DWORD langid, char* buf, DWORD sz, void* args)
{ (void)flags; (void)src; (void)msgid; (void)langid; (void)args;
  if (buf && sz > 0) buf[0] = '\0'; return 0; }

HANDLE STDCALL LoadLibraryW(LPCWSTR name)
{ (void)name; return NULL_HANDLE; }

HANDLE STDCALL LocalFree(HANDLE hmem)
{ (void)hmem; return NULL_HANDLE; /* NULL = success */ }

void* STDCALL AddVectoredExceptionHandler(DWORD first, void* handler)
{ (void)first; (void)handler; return PSEUDO_HANDLE; }

DWORD STDCALL RemoveVectoredExceptionHandler(void* handle)
{ (void)handle; return TRUE; }

/* --- msvcrt.dll stubs (cdecl, no @N decoration) --- */

unsigned long _beginthreadex(void* sec, unsigned stksz, unsigned (*start)(void*),
                             void* arg, unsigned flags, unsigned* tid)
{ (void)sec; (void)stksz; (void)start; (void)arg; (void)flags; (void)tid; return 0; }

void _endthreadex(unsigned retval)
{ (void)retval; }

void __setusermatherr(void* handler)
{ (void)handler; }

/* ================================================================
 * __imp__ pointers — satisfy dllimport-style references so the
 * linker never includes import-library archive members.
 * ================================================================ */

__asm__(
    ".section .data\n"

    ".globl __imp__CloseHandle@4\n"
    "__imp__CloseHandle@4:\n  .long _CloseHandle@4\n"

    ".globl __imp__CreateEventA@16\n"
    "__imp__CreateEventA@16:\n  .long _CreateEventA@16\n"

    ".globl __imp__CreateSemaphoreA@16\n"
    "__imp__CreateSemaphoreA@16:\n  .long _CreateSemaphoreA@16\n"

    ".globl __imp__DuplicateHandle@28\n"
    "__imp__DuplicateHandle@28:\n  .long _DuplicateHandle@28\n"

    ".globl __imp__GetCurrentProcess@0\n"
    "__imp__GetCurrentProcess@0:\n  .long _GetCurrentProcess@0\n"

    ".globl __imp__GetCurrentProcessId@0\n"
    "__imp__GetCurrentProcessId@0:\n  .long _GetCurrentProcessId@0\n"

    ".globl __imp__GetCurrentThread@0\n"
    "__imp__GetCurrentThread@0:\n  .long _GetCurrentThread@0\n"

    ".globl __imp__GetCurrentThreadId@0\n"
    "__imp__GetCurrentThreadId@0:\n  .long _GetCurrentThreadId@0\n"

    ".globl __imp__GetHandleInformation@8\n"
    "__imp__GetHandleInformation@8:\n  .long _GetHandleInformation@8\n"

    ".globl __imp__GetModuleHandleW@4\n"
    "__imp__GetModuleHandleW@4:\n  .long _GetModuleHandleW@4\n"

    ".globl __imp__GetProcessAffinityMask@12\n"
    "__imp__GetProcessAffinityMask@12:\n  .long _GetProcessAffinityMask@12\n"

    ".globl __imp__GetSystemTimeAsFileTime@4\n"
    "__imp__GetSystemTimeAsFileTime@4:\n  .long _GetSystemTimeAsFileTime@4\n"

    ".globl __imp__GetThreadContext@8\n"
    "__imp__GetThreadContext@8:\n  .long _GetThreadContext@8\n"

    ".globl __imp__GetThreadPriority@4\n"
    "__imp__GetThreadPriority@4:\n  .long _GetThreadPriority@4\n"

    ".globl __imp__GetTickCount@0\n"
    "__imp__GetTickCount@0:\n  .long _GetTickCount@0\n"

    ".globl __imp__IsDBCSLeadByteEx@8\n"
    "__imp__IsDBCSLeadByteEx@8:\n  .long _IsDBCSLeadByteEx@8\n"

    ".globl __imp__IsDebuggerPresent@0\n"
    "__imp__IsDebuggerPresent@0:\n  .long _IsDebuggerPresent@0\n"

    ".globl __imp__MultiByteToWideChar@24\n"
    "__imp__MultiByteToWideChar@24:\n  .long _MultiByteToWideChar@24\n"

    ".globl __imp__OpenProcess@12\n"
    "__imp__OpenProcess@12:\n  .long _OpenProcess@12\n"

    ".globl __imp__OutputDebugStringA@4\n"
    "__imp__OutputDebugStringA@4:\n  .long _OutputDebugStringA@4\n"

    ".globl __imp__QueryPerformanceCounter@4\n"
    "__imp__QueryPerformanceCounter@4:\n  .long _QueryPerformanceCounter@4\n"

    ".globl __imp__QueryPerformanceFrequency@4\n"
    "__imp__QueryPerformanceFrequency@4:\n  .long _QueryPerformanceFrequency@4\n"

    ".globl __imp__RaiseException@16\n"
    "__imp__RaiseException@16:\n  .long _RaiseException@16\n"

    ".globl __imp__ReleaseSemaphore@12\n"
    "__imp__ReleaseSemaphore@12:\n  .long _ReleaseSemaphore@12\n"

    ".globl __imp__ResetEvent@4\n"
    "__imp__ResetEvent@4:\n  .long _ResetEvent@4\n"

    ".globl __imp__ResumeThread@4\n"
    "__imp__ResumeThread@4:\n  .long _ResumeThread@4\n"

    ".globl __imp__SetEvent@4\n"
    "__imp__SetEvent@4:\n  .long _SetEvent@4\n"

    ".globl __imp__SetLastError@4\n"
    "__imp__SetLastError@4:\n  .long _SetLastError@4\n"

    ".globl __imp__SetProcessAffinityMask@8\n"
    "__imp__SetProcessAffinityMask@8:\n  .long _SetProcessAffinityMask@8\n"

    ".globl __imp__SetThreadContext@8\n"
    "__imp__SetThreadContext@8:\n  .long _SetThreadContext@8\n"

    ".globl __imp__SetThreadPriority@8\n"
    "__imp__SetThreadPriority@8:\n  .long _SetThreadPriority@8\n"

    ".globl __imp__SuspendThread@4\n"
    "__imp__SuspendThread@4:\n  .long _SuspendThread@4\n"

    ".globl __imp__TlsAlloc@0\n"
    "__imp__TlsAlloc@0:\n  .long _TlsAlloc@0\n"

    ".globl __imp__TlsSetValue@8\n"
    "__imp__TlsSetValue@8:\n  .long _TlsSetValue@8\n"

    ".globl __imp__TryEnterCriticalSection@4\n"
    "__imp__TryEnterCriticalSection@4:\n  .long _TryEnterCriticalSection@4\n"

    ".globl __imp__WaitForMultipleObjects@16\n"
    "__imp__WaitForMultipleObjects@16:\n  .long _WaitForMultipleObjects@16\n"

    ".globl __imp__WaitForSingleObject@8\n"
    "__imp__WaitForSingleObject@8:\n  .long _WaitForSingleObject@8\n"

    ".globl __imp__WideCharToMultiByte@32\n"
    "__imp__WideCharToMultiByte@32:\n  .long _WideCharToMultiByte@32\n"

    ".globl __imp__FormatMessageA@28\n"
    "__imp__FormatMessageA@28:\n  .long _FormatMessageA@28\n"

    ".globl __imp__LoadLibraryW@4\n"
    "__imp__LoadLibraryW@4:\n  .long _LoadLibraryW@4\n"

    ".globl __imp__LocalFree@4\n"
    "__imp__LocalFree@4:\n  .long _LocalFree@4\n"

    ".globl __imp__AddVectoredExceptionHandler@8\n"
    "__imp__AddVectoredExceptionHandler@8:\n  .long _AddVectoredExceptionHandler@8\n"

    ".globl __imp__RemoveVectoredExceptionHandler@4\n"
    "__imp__RemoveVectoredExceptionHandler@4:\n  .long _RemoveVectoredExceptionHandler@4\n"

    /* msvcrt cdecl stubs — no @N in symbol names */
    ".globl __imp___beginthreadex\n"
    "__imp___beginthreadex:\n  .long __beginthreadex\n"

    ".globl __imp___endthreadex\n"
    "__imp___endthreadex:\n  .long __endthreadex\n"

    ".globl __imp____setusermatherr\n"
    "__imp____setusermatherr:\n  .long ___setusermatherr\n"
);

#endif /* ROSCO_PHARLAP */
