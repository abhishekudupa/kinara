#!/usr/bin/env python

import re
import sys

vals = []

for rec in open('res245.csv', 'r'):
    match = re.search('(.*), (\d+), (\d+), (\d+)', rec)
    if not match:
        continue

    vals.append([match.group(1), int(match.group(2)), int(match.group(3)), int(match.group(4))])

relax = float(sys.argv[1]) if len(sys.argv) > 2 else 1.0

print sys.argv[0]
fvals = []
for rec in vals:
    betterFound = False
    for recp in vals:
        if recp[1] < relax * rec[1] and recp[2] < relax * rec[2] and recp[3] < relax * rec[3]:
            betterFound = True
            break
    if not betterFound:
        print rec[0], rec[1], rec[2], rec[3]

# print len(fvals)

