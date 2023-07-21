import subprocess
import sys

def call_child_scripts(arg, arg2=None):
    try:
        # Call file1.py with the argument

        if arg2 is None:
            subprocess.run(['python3', './scripts/data_process.py', arg], check=True)
            subprocess.run(['python3', './scripts/aero_plot.py', arg], check=True)
        else:
            subprocess.run(['python3', './scripts/data_process.py', arg, arg2], check=True)
            subprocess.run(['python3', './scripts/aero_plot.py', arg, arg2], check=True)

    except subprocess.CalledProcessError as e:
        print(f"Error executing the script: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 all_data.py <foldername> <optional: csv_path> ")
        sys.exit(1)
        
    if len(sys.argv) == 2:
        call_child_scripts(sys.argv[1])
    else:
        call_child_scripts(sys.argv[1], arg2)