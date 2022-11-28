import sys
import subprocess
import random
from datetime import datetime
from time import sleep


def run_repetitions():
    for i in range(N_REP):
        for executable in executables:
            if executable == executables[0]:
                process = subprocess.Popen([working_dir + executable, input_file, str(seed), "1"], stdout=subprocess.DEVNULL)
                ret_code = process.wait()
                if executable == executables[0] and ret_code != 0:
                    return
            else:
                for t in threads:
                    process = subprocess.Popen([working_dir + executable + t + ".exe", input_file, str(seed), "1"], stdout=subprocess.DEVNULL)
                    process.wait()


random.seed(datetime.now())

MAXINT = 2147483647

executables = ["sequential_astar.exe", "hdastar_message_passing_", "hdastar_shared_"]
threads = ["1", "4", "8", "12", "14", "16"]

if len(sys.argv) < 4:
    print("USAGE: python launcher.py N_SEEDS N_REPS WORKING_DIR INPUT_FILE")

N_SEEDS = int(sys.argv[1])
N_REP = int(sys.argv[2])
working_dir = sys.argv[3]
if working_dir[-1] != "/":
    working_dir = working_dir[:] + "/"
input_file = sys.argv[4]

for n in range(N_SEEDS):
    seed = random.randint(0, MAXINT)
    run_repetitions()


