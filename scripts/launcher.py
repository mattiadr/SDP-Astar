import sys
import subprocess
import random
from datetime import datetime
from time import sleep


def run_repetitions():
    for i in range(N_REP):
        for executable in executables:
            process = subprocess.Popen([working_dir + executable, input_file, str(seed), "1"])
            ret_code = process.wait()
            if executable == executables[0] and ret_code != 0:
                return


random.seed(datetime.now())

MAXINT = 2147483647

executables = ["sequential_astar.exe", "hdastar_message_passing.exe", "hdastar_shared.exe"]

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


