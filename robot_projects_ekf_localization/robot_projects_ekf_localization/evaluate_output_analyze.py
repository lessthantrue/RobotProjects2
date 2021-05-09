import matplotlib.pyplot as plt
import sys

def getLogOutputs(filename):
    outs = []
    with open(filename) as f:
        for line in f:
            vals = list(map(lambda x: float(x.strip()), line.split(",")))
            outs.append(-vals[1])
    return outs

plt.plot(getLogOutputs(sys.argv[1]))
plt.yscale('log')
plt.show()