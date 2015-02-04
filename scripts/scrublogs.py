#!/usr/bin/python3

import sys
import os
import itertools
import re

guard_opts = ['none', 'nonfalse', 'vardep', 'point']
update_opts = ['none', 'nonid', 'vardep']
state_update_opts = ['none', 'allsame', 'vardep']
quant_opts = ['none', 'unroll']

if (len(sys.argv) < 4):
    print('Usage: %s <dir-name> <csv-name> <to-value>' % sys.argv[0])
    sys.exit(1)

timeout_value = int(sys.argv[3])

for suffix in ['full', 'narrow']:
    csvfile = open(sys.argv[2] + '_' + suffix + '.csv', 'w')
    csvfile.write('Guard, Update, Location, Quantifiers, Time (s), # Iterations, Total SMT Time (s), Min SMT Time (uS), Max SMT Time (mS), Avg SMT Time (mS), # Assertions Start, # Assertions End\n')
    prod = itertools.product(guard_opts, update_opts, state_update_opts, quant_opts)
    for prod_tuple in prod:
        log_file_name = 'log_%s_%s_%s_%s_%s' % (prod_tuple[0], prod_tuple[1], prod_tuple[2], prod_tuple[3], suffix)
        print('Processing log file: %s' % log_file_name)
        log_file = open(os.path.join(sys.argv[1], log_file_name), 'r')
        log_data = log_file.read()
        timedout = re.search('CPU Time limit reached. Aborting with Timeout', log_data)
        solve_time = re.search('Solve Time:\s*(\d+)', log_data)
        solve_time = int(solve_time.group(1))
        init_asserts = re.search('Initial Asserts:\s*(\d+)', log_data)
        init_asserts = int(init_asserts.group(1))
        final_asserts = re.search('Final Asserts:\s*(\d+)', log_data)
        final_asserts = int(final_asserts.group(1))
        num_iterations = re.search('Num Iterations:\s*(\d+)', log_data)
        num_iterations = int(num_iterations.group(1))
        total_smt_time = re.search('Total SMT Time:\s*(\d+)', log_data)
        total_smt_time = int(total_smt_time.group(1))
        min_smt_time = re.search('Min SMT Time:\s*(\d+)', log_data)
        min_smt_time = int(min_smt_time.group(1))
        max_smt_time = re.search('Max SMT Time:\s*(\d+)', log_data)
        max_smt_time = int(max_smt_time.group(1))
        avg_smt_time = total_smt_time / num_iterations
        final_bound = re.search('Final Bound:\s*(\d+)', log_data)
        final_bound = int(final_bound.group(1))

        csvfile.write('%s, %s, %s, %s, %s, %d, %d, %d, %d, %d, %d, %d\n' % (prod_tuple[0],
                                                                            prod_tuple[1],
                                                                            prod_tuple[2],
                                                                            prod_tuple[3],
                                                                            timeout_value if timedout else solve_time,
                                                                            num_iterations,
                                                                            total_smt_time / 1000000,
                                                                            min_smt_time,
                                                                            max_smt_time / 1000,
                                                                            avg_smt_time / 1000,
                                                                            init_asserts,
                                                                            final_asserts))
    csvfile.close()
