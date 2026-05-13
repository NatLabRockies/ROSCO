/*
 * veh_stubs.c — Stub implementations of Vectored Exception Handler functions
 *
 * Phar Lap ETS's KERNEL32.dll does not implement AddVectoredExceptionHandler
 * or RemoveVectoredExceptionHandler (these were added in Windows XP SP1).
 * MinGW's DWARF exception handling setup calls these during DLL initialization,
 * causing load failure on Phar Lap.
 *
 * By providing local implementations, the linker resolves these symbols from
 * this object file instead of the KERNEL32.dll import library. The functions
 * never appear in the DLL's import table, so Phar Lap never needs to provide them.
 *
 * The stubs return non-NULL / non-zero to indicate "success" to the caller.
 * DWARF exception handling still works for C++ exceptions that are thrown and
 * caught within the DLL — it just won't catch hardware exceptions (access
 * violations, etc.) which wouldn't be recoverable on Phar Lap anyway.
 *
 * Build: compile this file into every DLL that needs to load on Phar Lap.
 *        The object must be listed BEFORE -lkernel32 on the link line.
 */

#ifdef ROSCO_PHARLAP

/* --- function stubs (stdcall, matching WINAPI) --- */

void* __attribute__((stdcall))
AddVectoredExceptionHandler(unsigned long First, void* Handler)
{
    (void)First;
    (void)Handler;
    return (void*)1;  /* non-NULL = "registered successfully" */
}

unsigned long __attribute__((stdcall))
RemoveVectoredExceptionHandler(void* Handle)
{
    (void)Handle;
    return 1;  /* non-zero = "removed successfully" */
}

/* --- __imp__ pointers (satisfies dllimport-style references) ---
 * MinGW's DWARF runtime calls these via __imp__FunctionName@N (the
 * dllimport indirection pointer).  By providing these symbols here,
 * the linker won't pull in the import-library archive member from
 * libkernel32.a, so no import-table entry is created in the DLL. */

__asm__(
    ".section .data\n"
    ".globl __imp__AddVectoredExceptionHandler@8\n"
    "__imp__AddVectoredExceptionHandler@8:\n"
    "  .long _AddVectoredExceptionHandler@8\n"
    ".globl __imp__RemoveVectoredExceptionHandler@4\n"
    "__imp__RemoveVectoredExceptionHandler@4:\n"
    "  .long _RemoveVectoredExceptionHandler@4\n"
);

#endif /* ROSCO_PHARLAP */
