import matplotlib.pyplot as plt
import numpy as np
import csv
import pandas as pd
import sys
import os


def main():
    file_list = []
    if len(sys.argv) > 1:
        file_list.append(sys.argv[1])
    else:
        # go through all .csv files
        for filename in os.listdir("."):
            if filename.endswith(".csv"):
                file_list.append(filename)
            else:
                continue

    plt.style.use('classic')

    x = []
    y = []

    for filename in file_list:
        df = pd.read_csv(filename)
        df.values
        df.as_matrix(columns=None)
        result = df.values.tolist()

        for i in result:
            x.append(i[0])
            y.append(i[1])

    plt.xlim((0, 14))
    plt.ylim((0, 14))
    plt.xlabel('lidar')
    plt.ylabel('odom')
    plt.scatter(x,y)
    plt.savefig(filename+'.png',dpi=1500)
    plt.show()

if __name__ == '__main__':
    main()
