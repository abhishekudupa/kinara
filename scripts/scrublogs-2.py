#!/usr/bin/python3

import sys
import os
import itertools
import re

if (len(sys.argv) < 4):
    print('Usage: %s <dir-name> <csv-name> <to-value>' % sys.argv[0])
    sys.exit(1)

timeout_value = int(sys.argv[3])
csvfile = open(sys.argv[2], 'w')

for log_filename in os.listdir(sys.argv[1]):
    if log_filename.startswith('log') and not log_filename.endswith('.err'):
        log_data = open(os.path.join(sys.argv[1], log_filename), 'r').read()
        params = re.search('@@@@@@@(.*)@@@@@@@', log_data)

        timedout = re.search('CPU Time limit reached. Aborting with Timeout', log_data)
        solve_time = re.search('Solve Time:\s*(\d+)', log_data)
        solve_time = int(solve_time.group(1)) if solve_time else 0
        init_asserts = re.search('Initial Asserts:\s*(\d+)', log_data)
        init_asserts = int(init_asserts.group(1)) if init_asserts else 0
        final_asserts = re.search('Final Asserts:\s*(\d+)', log_data)
        final_asserts = int(final_asserts.group(1)) if final_asserts else 0
        num_iterations = re.search('Num Iterations:\s*(\d+)', log_data)
        num_iterations = int(num_iterations.group(1)) if num_iterations else 0
        total_smt_time = re.search('Total SMT Time:\s*(\d+)', log_data)
        total_smt_time = int(total_smt_time.group(1)) if total_smt_time else 0
        min_smt_time = re.search('Min SMT Time:\s*(\d+)', log_data)
        min_smt_time = int(min_smt_time.group(1)) if min_smt_time else 0
        max_smt_time = re.search('Max SMT Time:\s*(\d+)', log_data)
        max_smt_time = int(max_smt_time.group(1)) if max_smt_time else 0
        avg_smt_time = total_smt_time / num_iterations if num_iterations != 0 else 0
        final_bound = re.search('Final Bound:\s*(\d+)', log_data)
        final_bound = int(final_bound.group(1)) if final_bound else 0

        if params:
            params = params.group(1)
            csvfile.write('%s, %s, %d, %d, %d, %d, %d, %d, %d\n' % (params,
                                                                    timeout_value if timedout else solve_time,
                                                                    num_iterations,
                                                                    total_smt_time / 1000000,
                                                                    min_smt_time,
                                                                    max_smt_time / 1000,
                                                                    avg_smt_time / 1000,
                                                                    init_asserts,
                                                                    final_asserts))
        else:
            # Could not parse parameters
            print('Could not parse log file: %s' % log_filename)

csvfile.close()

