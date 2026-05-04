// read_config_files — load DISCON.IN or DISCON.toml + Cp/Ct/Cq tables.
//
// Called on first timestep (iStatus == 0) and on warm restart (iStatus == -9).
// Reads the config filename from LocalVar.ACC_INFILE (std::string).
// Auto-detects format by file extension (.toml → TOML, else legacy DISCON.IN).

#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"
#include "../include/vit_translated.h"
#include <string>

// GetRoot: strip extension from a filename
// e.g. "/path/to/Case01.outb" → "/path/to/Case01"
std::string GetRoot(const std::string& filename) {
    auto dot = filename.rfind('.');
    if (dot == std::string::npos) return filename;
    return filename.substr(0, dot);
}

// Helper: get file extension (including the dot)
static std::string getExtension(const std::string& filename) {
    auto dot = filename.rfind('.');
    if (dot == std::string::npos) return "";
    return filename.substr(dot);
}

// Helper: get parent directory path (with trailing separator)
static std::string getParentPath(const std::string& filename) {
    auto sep = filename.find_last_of("/\\");
    if (sep == std::string::npos) return "./";
    return filename.substr(0, sep + 1);
}

void read_config_files(ControlParameters& CntrPar, LocalVariables& LocalVar,
                       PerformanceData& PerfData) {
    const std::string& filename = LocalVar.ACC_INFILE;

    // Reset parameters to defaults before re-reading
    CntrPar = ControlParameters{};

    std::string ext = getExtension(filename);
    bool is_toml = (ext == ".toml" || ext == ".TOML");

    if (is_toml) {
        CntrPar.load_from_toml(filename.c_str());
    } else {
        // Directory containing the config file — used to resolve relative paths
        std::string priPath = getParentPath(filename);
        ReadControlParameterFileSub(CntrPar, LocalVar, filename.c_str(), priPath.c_str());
    }

    // Load rotor performance tables (required when WE_Mode > 0)
    PerfData = PerformanceData{};
    if (CntrPar.WE_Mode > 0) {
        ReadCpFile(CntrPar, PerfData);
    }
}
