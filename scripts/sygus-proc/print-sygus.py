#!/usr/bin/env python
# print-sygus <log-file>

import re
import sys

logFile = sys.argv[1]
logFile = open(logFile, 'r')
logFile = logFile.readlines()

def writeSL(funSignature, assigns):
    print '; %s' % (funSignature[:len(funSignature) - 6])
    funSignature = re.search('\((.*)\) -> {', funSignature)
    funSignature = funSignature.group(1)
    funSignature = funSignature.split()

    funName = funSignature[0]
    funArgs = funSignature[1:]

    # print funName, funArgs
    # print assigns
    for assign in assigns:
        print '(constraint (= (%s %s) %s))' % (funName, ' '.join(assign[0]), assign[1])

i = 0
while i < len(logFile):
    line = logFile[i]
    i += 1

    if not line.startswith('Model for uninterpreted function:'):
        continue

    funSignature = logFile[i]
    i += 1
    assigns = []
    while not logFile[i].startswith('}'):
        linePrime = logFile[i]
        i += 1

        args = linePrime.split()
        retVal = args[-1]
        args = args[:len(args) - 2]
        assigns.append([args, retVal])

    writeSL(funSignature, assigns)
    print

