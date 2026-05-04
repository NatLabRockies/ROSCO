#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"
#include <cstdio>
#include <cstring>

#ifndef ROSCO_NO_EXT_DLL
// --- Dynamic library loading: only available on desktop platforms ---

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

// Bladed DLL legacy interface — function pointer typedef
typedef void (*bladed_dll_proc_t)(float*, int*, char*, char*, char*);

void ExtController(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, ExtControlType& ExtDLL) {
#ifdef _WIN32
    static HMODULE dll_handle = nullptr;
#else
    static void* dll_handle = nullptr;
#endif
    static bladed_dll_proc_t dll_proc = nullptr;

    const int max_avr_entries = 2000;
    const char* ExtRootName = "external_control";

    // Initialize strings for external controller
    int aviFAIL = 0;

    // Build null-terminated C strings from Fortran CHARACTER fields
    // DLL_InFile
    char accINFILE[1024];
    int infile_len = 0;
    while (infile_len < 1023 && CntrPar.DLL_InFile[infile_len] != ' ' && CntrPar.DLL_InFile[infile_len] != '\0') {
        accINFILE[infile_len] = CntrPar.DLL_InFile[infile_len];
        infile_len++;
    }
    accINFILE[infile_len] = '\0';

    // OUTNAME
    char avcOUTNAME[128];
    int outname_len = (int)strlen(ExtRootName);
    memcpy(avcOUTNAME, ExtRootName, outname_len);
    avcOUTNAME[outname_len] = '\0';

    // MSG buffer
    char avcMSG[1025];
    memset(avcMSG, 0, sizeof(avcMSG));

    // First call: load dynamic library
    if (LocalVar.iStatus == 0) {
        // Build null-terminated filename
        char dll_filename[1024];
        int fn_len = 0;
        while (fn_len < 1023 && CntrPar.DLL_FileName[fn_len] != ' ' && CntrPar.DLL_FileName[fn_len] != '\0') {
            dll_filename[fn_len] = CntrPar.DLL_FileName[fn_len];
            fn_len++;
        }
        dll_filename[fn_len] = '\0';

        // Build null-terminated proc name
        char dll_procname[1024];
        int pn_len = 0;
        while (pn_len < 1023 && CntrPar.DLL_ProcName[pn_len] != ' ' && CntrPar.DLL_ProcName[pn_len] != '\0') {
            dll_procname[pn_len] = CntrPar.DLL_ProcName[pn_len];
            pn_len++;
        }
        dll_procname[pn_len] = '\0';

        printf("ROSCO is calling an external dynamic library for control input:\n");
        printf("DLL_FileName: %s\n", dll_filename);
        printf("DLL_InFile: %s\n", accINFILE);
        printf("DLL_ProcName: %s\n", dll_procname);

#ifdef _WIN32
        dll_handle = LoadLibraryA(dll_filename);
        if (!dll_handle) {
            fprintf(stderr, "ExtController: The dynamic library %s could not be loaded.\n", dll_filename);
            return;
        }
        dll_proc = (bladed_dll_proc_t)GetProcAddress(dll_handle, dll_procname);
        if (!dll_proc) {
            fprintf(stderr, "ExtController: The procedure %s could not be loaded.\n", dll_procname);
            return;
        }
#else
        dll_handle = dlopen(dll_filename, RTLD_LAZY);
        if (!dll_handle) {
            fprintf(stderr, "ExtController: The dynamic library %s could not be loaded.\n", dll_filename);
            return;
        }
        dll_proc = (bladed_dll_proc_t)dlsym(dll_handle, dll_procname);
        if (!dll_proc) {
            fprintf(stderr, "ExtController: The procedure %s could not be loaded.\n", dll_procname);
            return;
        }
#endif

        printf("Library loaded successfully\n");
    }

    // Copy avrSWAP to ExtDLL's swap array
    for (int i = 0; i < max_avr_entries; i++) {
        ExtDLL.avrSWAP[i] = avrSWAP[i];
    }

    // Set length parameters (0-based: records 49,50,51 → indices 48,49,50)
    ExtDLL.avrSWAP[48] = (float)(sizeof(avcMSG));           // Record 49: max MSG length
    ExtDLL.avrSWAP[49] = (float)(infile_len + 1);           // Record 50: INFILE length
    ExtDLL.avrSWAP[50] = (float)(outname_len + 1);          // Record 51: OUTNAME length

    // Call the external DLL
    dll_proc(ExtDLL.avrSWAP.data(), &aviFAIL, accINFILE, avcOUTNAME, avcMSG);

    // Check for errors from the DLL
    if (aviFAIL < 0) {
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "ExtController:%s", avcMSG);
        printf("%s\n", tmp);
    }
}

#else
// --- ROSCO_NO_EXT_DLL: stub for Phar Lap / static builds ---

void ExtController(float* /*avrSWAP*/, const ControlParameters& /*CntrPar*/,
                   LocalVariables& /*LocalVar*/, ExtControlType& /*ExtDLL*/) {
    // External controller not available in this build.
    // DLL_Mode must be 0 in DISCON.IN.
}

#endif // ROSCO_NO_EXT_DLL
