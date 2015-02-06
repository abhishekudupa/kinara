#!/usr/bin/env python

import re
import sys

vals = []

for rec in open('res245.csv', 'r'):
    match = re.search('(.*), (\d+), (\d+), (\d+)', rec)
    if not match:
        continue

    vals.append([match.group(1), int(match.group(2)), int(match.group(3)), int(match.group(4))])

limRank = int(sys.argv[1]) if len(sys.argv) > 1 else 50
fv2 = sorted(vals, cmp=lambda x, y: cmp(x[1], y[1]))[:limRank]
fv4 = sorted(vals, cmp=lambda x, y: cmp(x[2], y[2]))[:limRank]
fv5 = sorted(vals, cmp=lambda x, y: cmp(x[3], y[3]))[:limRank]

fvals = []

print sys.argv[0]
for rec in fv2:
    if rec in fv4 and rec in fv5:
        print rec[0], rec[1], rec[2], rec[3]
        fvals.append(rec)

# print len(fvals)

