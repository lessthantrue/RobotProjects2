import matplotlib.pyplot as plt
import sys
import numpy as np
import os

def csvLineToFloatList(line):
    return list(map(lambda x: float(x.strip()), line.split(",")))

def getCsvColumn(filename, column_no):
    outs = []
    with open(filename) as f:
        for line in f:
            l = csvLineToFloatList(line)
            outs.append(l[column_no])
    return np.array(outs)

def plotFile(fname):
    log_probs = getCsvColumn(fname, 2)
    times = getCsvColumn(fname, 0)
    plt.plot(times, -log_probs, label=fname.split("/")[-1])

fnames = sys.argv[1:]
for fname in fnames:
    if os.path.isfile(fname):
        plotFile(fname)
    elif os.path.isdir(fname):
        for f in os.listdir(fname):
            plotFile(os.path.join(fname, f))
    
plt.yscale('log')
plt.legend()
plt.show()