import argparse
import glob
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--generic", action="store_true",
                         help="Write a generic \"DISCON_LIB_PATH\" placeholder instead of this machine's actual discon_lib_path")
    args = parser.parse_args()

    if args.generic:
        discon_lib_path = "DISCON_LIB_PATH"
    else:
        from rosco import discon_lib_path

    this_dir   = os.path.dirname(os.path.abspath(__file__))
    servo_list = glob.glob(os.path.join(this_dir, '**/*Servo*.dat'),recursive=True)

    for ifile in servo_list:
        # Read in current ServoDyn file
        with open(ifile, "r") as f:
            lines = f.readlines()

        # Write correction
        with open(ifile, "w") as f:
            for line in lines:
                if line.find("DLL_FileName") >= 0:
                    f.write(f"\"{discon_lib_path}\"    DLL_FileName - Name/location of the dynamic library (.dll [Windows] or .so [Linux]) in the Bladed-DLL format (-) [used only with Bladed Interface]\n")
                else:
                    f.write(line)
                
