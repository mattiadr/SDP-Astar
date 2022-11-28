import sys
import subprocess
import random
from datetime import datetime


def run_repetitions():
    for executable in executables:
        if executable == executables[0]:
            process = subprocess.Popen([working_dir + executable, input_file, str(seed), str(N_SEEDS), str(N_REP)], stdout=subprocess.DEVNULL)
            ret_code = process.wait()
        else:
            for t in threads:
                process = subprocess.Popen([working_dir + executable + t + ".exe", input_file, str(seed), str(N_SEEDS), str(N_REP)], stdout=subprocess.DEVNULL)
                process.wait()


random.seed(datetime.now())
MAXINT = 2147483647

executables = ["sequential_astar.exe", "hdastar_message_passing_", "hdastar_shared_"]
# threads = ["1", "4", "8", "12", "14", "16"]
threads = ["16"]

if len(sys.argv) < 4:
    print("USAGE: python launcher.py N_SEEDS N_REPS WORKING_DIR INPUT_FILE")

N_SEEDS = int(sys.argv[1])
N_REP = int(sys.argv[2])
working_dir = sys.argv[3]
if working_dir[-1] != "/":
    working_dir = working_dir[:] + "/"
input_file = sys.argv[4]

seed = random.randint(0, MAXINT)
run_repetitions()


