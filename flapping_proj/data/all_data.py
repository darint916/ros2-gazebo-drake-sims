import subprocess
import sys

def call_child_scripts(arg, arg2=""):
    try:
        # Call file1.py with the argument
        subprocess.run(['python3', './scripts/aero_plot.py', arg, arg2], check=True)

        # Call file2.py with the same argument
        subprocess.run(['python3', './scripts/data_process.py', arg, arg2], check=True)

    except subprocess.CalledProcessError as e:
        print(f"Error executing the script: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 all_data.py <foldername> <optional: csv_path> ")
        sys.exit(1)
    if len(sys.argv) == 2:
        arg2 = ""
    else:
        arg2 = sys.argv[2]
    call_child_scripts(sys.argv[1], arg2)