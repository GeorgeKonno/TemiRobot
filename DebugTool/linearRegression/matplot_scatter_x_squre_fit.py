#!/usr/bin/env python 
# coding:utf-8 
##copyright from 吴恩达,二阶最小二乘法,模板库

import matplotlib.pyplot as plt
import numpy as np
import csv
import pandas as pd
import sys
import os
from scipy.optimize import leastsq 
# 二次函数的标准形式 
def func(params, x): 
    a, b, c = params 
    return a * x * x + b * x + c 
# 误差函数，即拟合曲线所求的值与实际值的差 
def error(params, x, y): 
    return func(params, x) - y 
# 对参数求解 
def slovePara(): 
    p0 = [10, 10, 10] 
    Para = leastsq(error, p0, args=(X, Y)) 
    return Para 
# 输出最后的结果 
def solution(filename): 
    Para = slovePara() 
    a, b, c = Para[0] 
    print "a=",a," b=",b," c=",c 
    print "cost:" + str(Para[1]) 
    print "求解的曲线是:" 
    print("y="+str(round(a,2))+"x*x+"+str(round(b,2))+"x+"+str(c)) 
    plt.figure(figsize=(8,6)) 
    plt.scatter(X, Y, color="green", label="sample data", linewidth=2) 
    x_new=np.linspace(0,12,100)
    y_new=a*x_new*x_new+b*x_new+c 
    plt.plot(x_new,y_new,color="red",label="solution line",linewidth=2) 
    plt.legend() 
    plt.savefig(filename+'.png',dpi=1500)
    plt.show() 
    
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
        x_1 = []
        y_1 = []
        for i in result:
            x.append(i[0])
            y.append(i[1])
            x_1.append(i[0])
            y_1.append(i[1])
        #简单边界清除离散点
        #delete_ = True
        #z = 0
        #while delete_:
            #if z < len(x_1):
                #if x_1[z]/y_1[z]>1.5 or y_1[z]/x_1[z]>1.5:
                    #print z
                    #print x_1[z],y_1[z]
                    #x_1.pop(z)
                    #y_1.pop(z)
                #else:
                    #z = z + 1
            #else:
                #delete_ = False
        X = np.array(x_1) 
        Y = np.array(y_1) 
        solution(filename)


    plt.xlim((0, 14))
    plt.ylim((0, 14))
    plt.xlabel('lidar')
    plt.ylabel('odom')
    plt.scatter(x,y)
    plt.savefig(filename+'total'+'.png',dpi=1500)
    plt.show()

if __name__ == '__main__':
    main()
