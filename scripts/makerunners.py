#!/usr/bin/python

import sys
import itertools
import os, stat
import pprint

guard_opts = ['none', 'nonfalse', 'vardep', 'point', 'pointnoprefalltrue']
update_opts = ['none', 'nonid', 'vardep', 'point']
state_opts = ['none', 'allsame', 'vardep']
prio_opts = ['none', 'simple', 'coverage' ]
restart_opts = [ 'restart', 'none' ]
search_opts = ['binary', 'binaryminimal', 'none']

num_counterexamples = 2;
num_simple_prio_counterexamples = 2;
coverage_desired = 1;
num_coverage_counterexamples = 1;
bound = 16
missing_transitions = 'D_BUSY_WB C_II_SENDACK'
timeout_value = 3600

# assumes options are given in the same order as above
def opts_to_option_string(opts):
    retval = ''
    guard_opt = opts[0]
    update_opt = opts[1]
    state_opt = opts[2]
    prio_opt = opts[3]
    restart_opt = opts[4]
    search_opt = opts[5]
    if (guard_opt == 'nonfalse'):
        retval += ' -g nonfalse'
    elif (guard_opt == 'vardep'):
        retval += ' -g vardep'
    elif (guard_opt == 'point'):
        retval += ' -g point'
    elif (guard_opt == 'pointnoprefalltrue'):
        retval += ' --no-prefer-all-true'
    elif (guard_opt == 'none'):
        retval += ' -g none'

    retval += ' -u %s' % update_opt
    retval += ' -s %s' % state_opt
    retval += ' -p %s' % prio_opt
    retval += (' -r' if restart_opt == 'restart' else '')

    if (search_opt == 'binary'):
        retval += ' --binary-search'
    if (search_opt == 'binaryminimal'):
        retval += ' --binary-search --minimal-solution'

    if (prio_opt == 'none'):
        retval += (' -x %d' % num_counterexamples)
    elif (prio_opt == 'simple'):
        retval += (' -x %d' % num_simple_prio_counterexamples)
    elif (prio_opt == 'coverage'):
        retval += (' -c %f -x %d' % (coverage_desired, num_coverage_counterexamples))

    retval += (' -q -b %d' % bound)
    retval += (' --missing-transitions %s' % missing_transitions)
    retval += (' -t %d' % timeout_value)
    return retval;


def print_usage():
    print('Usage:')
    print('%s <exec_name> <log-dir> <timeout> <num-machines> <parallel-jobs-per-machine>' % (sys.argv[0]))
    sys.exit(1)

if len(sys.argv) < 6:
    print_usage()

num_machines = int(sys.argv[4])
parallel_jobs_per_machine = int(sys.argv[5])
exec_name = sys.argv[1]
log_dir = sys.argv[2]
timeout_value = int(sys.argv[3])

prod = itertools.product(guard_opts, update_opts, state_opts, prio_opts, restart_opts, search_opts)
prod_list = list(prod)
num_entries = len(prod_list)
job_lists = []
for i in range(num_machines):
    job_lists.append([])

last_end_index = 0
current_machine = 0
jobs_per_machine_low = num_entries / num_machines
jobs_per_machine_high = (num_entries / num_machines) + 1
jobs_per_machine_switch_at = num_machines - (num_entries % num_machines)

for cur_machine in range(num_machines):
    jobs_for_this_machine = jobs_per_machine_low
    if (cur_machine >= jobs_per_machine_switch_at):
        jobs_for_this_machine = jobs_per_machine_high

    job_lists[cur_machine].extend(prod_list[last_end_index:(last_end_index + jobs_for_this_machine)])
    last_end_index += jobs_for_this_machine

# expand the job lists
expanded_job_lists = []
job_counter = 0
for i in range(len(job_lists)):
    cur_job_list = job_lists[i]
    expanded_job_list = []
    for job in cur_job_list:
        log_file_name = ('log_%s_%d' % (os.path.basename(exec_name), job_counter))
        command_line_str = '%s %s' % (exec_name, opts_to_option_string(job))
        expanded_job_list.append((command_line_str, log_file_name))
        job_counter += 1

    expanded_job_lists.append(expanded_job_list)

print('Generated %d jobs in all, writing scripts...' % job_counter)

for i in range(len(expanded_job_lists)):
    cur_script_name = 'runner%d.sh' % (i+1)
    cur_script = open(cur_script_name, 'w')
    cur_script.write('#!/bin/bash\n\n')
    cur_script.write('rm -rf %s\n\n' % log_dir)
    cur_script.write('mkdir %s\n\n' % log_dir)

    my_job_list = expanded_job_lists[i]
    num_job_batches = len(my_job_list) / parallel_jobs_per_machine
    if (len(my_job_list) % parallel_jobs_per_machine != 0):
        num_job_batches += 1

    current_batch = 1
    for j in range(len(my_job_list)):
        cur_job = my_job_list[j]
        if (j != 0 and (j % parallel_jobs_per_machine) == 0):
            cur_script.write('\necho "Waiting for job batch %d of %d to complete..."\n' % (current_batch, num_job_batches))
            cur_script.write('wait\n')
            cur_script.write('echo "Job batch %d complete!"\n\n' % current_batch)
            current_batch += 1
        cur_script.write('%s > %s/%s 2>%s &\n' % (cur_job[0], log_dir, cur_job[1], cur_job[1] + '.errors'))

    cur_script.write('\necho "Waiting for job batch %d of %d to complete..."\n' % (current_batch, num_job_batches))
    cur_script.write('wait\n')
    cur_script.write('echo "Job batch %d complete!"\n\n' % current_batch)
    cur_script.close();

    file_stat = os.stat(cur_script_name)
    os.chmod(cur_script_name, file_stat.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
