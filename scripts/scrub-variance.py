#!/usr/bin/python3

import math
import sys
import os
import itertools
import re

if (len(sys.argv) < 4):
    print('Usage: %s <dir-name> <csv-name>' % sys.argv[0])
    sys.exit(1)

vals = {}
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

        if params and not timedout:
            params = params.group(1)
            if params not in vals:
                vals[params] = []
            vals[params].append([solve_time,
                                 num_iterations,
                                 float(total_smt_time) / 1000000,
                                 min_smt_time,
                                 float(max_smt_time) / 1000,
                                 float(avg_smt_time) / 1000,
                                 init_asserts,
                                 final_asserts,
                                 log_filename])
        elif timedout:
            print('Log file timed out: %s' % log_filename)
        else:
            # Could not parse parameters
            print('Could not parse log file: %s' % log_filename)

def median(l):
    mean_index = int(len(l) / 2)
    return sorted(l)[mean_index]

def mean(l):
    return float(sum(l)) / len(l)

def stddev(l):
    lmean = mean(l)
    var_list = [(x - lmean) ** 2 for x in l]
    s2 = float(sum(var_list)) / (len(l) - 1)
    return math.sqrt(s2)

csvfile = open(sys.argv[2], 'w')
csvfile.write('Guard Heuristic, Guard Do Not Prefer Alltrue, Update Heuristic, ' +
              'Location Update Heuristic, Priority Heuristic, TP Restart, ' +
              'Binary Search, Minimize Solution, ' +
              '#Completed iterations, ' +
              'Solve Time (Median), Solve Time (Mean), Solve Time (StdDev), ' +
              '#Iterations (Median), #Iterations (Mean), #Iterations (StdDev), ' +
              'Total SMT Time (Median), Total SMT Time (Mean), Total SMT Time (StdDev), ' +
              'Final Asserts (Median), Final Asserts (Mean), Final Asserts (StdDev)' +
              '\n')

for params in vals:
    pvals = vals[params]

    num_successful_iterations = len(pvals)

    solve_times = [x[0] for x in pvals]
    solve_time_median = median(solve_times)
    solve_time_mean = mean(solve_times)
    solve_time_stddev = stddev(solve_times)

    iter_counts = [x[1] for x in pvals]
    iter_count_median = median(iter_counts)
    iter_count_mean = mean(iter_counts)
    iter_count_stddev = stddev(iter_counts)

    total_smt_times = [x[2] for x in pvals]
    total_smt_median = median(total_smt_times)
    total_smt_mean = mean(total_smt_times)
    total_smt_stddev = stddev(total_smt_times)

    final_asserts = [x[7] for x in pvals]
    final_asserts_median = median(final_asserts)
    final_asserts_mean = mean(final_asserts)
    final_asserts_stddev = stddev(final_asserts)

    csvfile.write('%s, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n' %
                     ( params, num_successful_iterations,
                       solve_time_median, solve_time_mean, solve_time_stddev,
                       iter_count_median, iter_count_mean, iter_count_stddev,
                       total_smt_median, total_smt_mean, total_smt_stddev,
                       final_asserts_median, final_asserts_mean, final_asserts_stddev ))

csvfile.close()

