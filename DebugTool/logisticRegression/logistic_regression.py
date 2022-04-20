# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 09:28:21 2020

@author: gliu
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy.random
import time
from sklearn import preprocessing as pp
path = '/home/gliu/Desktop/logistic_regression.txt'
pdData = pd.read_csv(path, header=None, names=['Exam 1', 'Exam 2', 'Admitted'])
pdData.head()
positive = pdData[pdData['Admitted'] == 1] # returns the subset of rows such Admitted = 1, i.e. the set of *positive* examples
negative = pdData[pdData['Admitted'] == 0] # returns the subset of rows such Admitted = 0, i.e. the set of *negative* examples
fig, ax = plt.subplots(figsize=(10,5))
x1 = list(positive['Exam 1'])
y1 = list(positive['Exam 2'])
x2 = list(negative['Exam 1'])
y2 = list(negative['Exam 2'])
ax.scatter(x1, y1, s=30, c='b', marker='o', label='Admitted')
ax.scatter(x2, y2, s=30, c='r', marker='x', label='Not Admitted')
ax.legend()
ax.set_xlabel('Exam 1 Score')
ax.set_ylabel('Exam 2 Score')
pdData.insert(0, 'Ones', 1) 
STOP_ITER = 0
STOP_COST = 1
STOP_GRAD = 2
cols = 4
n=100


def sigmoid(z):
        return 1 / (1 + np.exp(-z))    
#nums = np.arange(-10, 10, step=1) #creates a vector containing 20 equally spaced values from -10 to 10
#fig, ax = plt.subplots(figsize=(12,4))
#ax.plot(nums, sigmoid(nums), 'r')
        
def model(X, theta):
        return sigmoid(np.dot(X, theta.T))
        
def cost(X, y, theta):
    left = np.multiply(-y, np.log(model(X, theta)))
    right = np.multiply(1 - y, np.log(1 - model(X, theta)))
    return np.sum(left - right) / (len(X))

def gradient(X, y, theta):
    grad = np.zeros(theta.shape)  # （1,3）
    error = (model(X, theta)- y).ravel()
    for j in range(len(theta.ravel())): #for each parmeter
        term = np.multiply(error, X[:,j])
        grad[0, j] = np.sum(term) / len(X)    
    return grad

def stopCriterion(type, value, threshold):
    if type == STOP_ITER:        
        return value > threshold
    elif type == STOP_COST:      
        return abs(value[-1]-value[-2]) < threshold
    elif type == STOP_GRAD:      
        return np.linalg.norm(value) < threshold

def shuffleData(data):
    np.random.shuffle(data)
    cols = data.shape[1]
    X = data[:, 0:cols-1]
    y = data[:, cols-1:]
    return X, y

def descent(data, theta, batchSize, stopType, thresh, alpha):
    #梯度下降求解    
    init_time = time.time()
    i = 0 # 迭代次数
    k = 0 # batch
    X, y = shuffleData(data)
    grad = np.zeros(theta.shape) # 计算的梯度
    costs = [cost(X, y, theta)] # 损失值   
    while True:
        grad = gradient(X[k:k+batchSize], y[k:k+batchSize], theta)
        k += batchSize #取batch数量个数据
        if k >= n: 
            k = 0 
            X, y = shuffleData(data) #重新洗牌
        theta = theta - alpha*grad # 参数更新
        costs.append(cost(X, y, theta)) # 计算新的损失
        i += 1 
        if stopType == STOP_ITER:       value = i
        elif stopType == STOP_COST:     value = costs
        elif stopType == STOP_GRAD:     value = grad
        if stopCriterion(stopType, value, thresh): break    
    return theta, i-1, costs, grad, time.time() - init_time    

def runExpe(data, theta, batchSize, stopType, thresh, alpha):
    #import pdb; pdb.set_trace();
    theta, iter, costs, grad, dur = descent(data, theta, batchSize, stopType, thresh, alpha)
    name = "Original" if (data[:,1]>2).sum() > 1 else "Scaled"
    name += " data - learning rate: {} - ".format(alpha)
    if batchSize==n: strDescType = "Gradient"
    elif batchSize==1:  strDescType = "Stochastic"
    else: strDescType = "Mini-batch ({})".format(batchSize)
    name += strDescType + " descent - Stop: "
    if stopType == STOP_ITER: strStop = "{} iterations".format(thresh)
    elif stopType == STOP_COST: strStop = "costs change < {}".format(thresh)
    else: strStop = "gradient norm < {}".format(thresh)
    name += strStop
    print ("***{}\nTheta: {} - Iter: {} - Last cost: {:03.2f} - Duration: {:03.2f}s".format(
        name, theta, iter, costs[-1], dur))
    fig, ax = plt.subplots(figsize=(12,4))
    ax.plot(np.arange(len(costs)), costs, 'r')
    ax.set_xlabel('Iterations')
    ax.set_ylabel('Cost')
    ax.set_title(name.upper() + ' - Error vs. Iteration')
    return theta

def predict(X, theta):
    return [1 if x >= 0.5 else 0 for x in model(X, theta)]



# in a try / except structure so as not to return an error if the block si executed several times
# set X (training data) and y (target variable)
orig_data = pdData.as_matrix() 
# convert the Pandas seful for further computations
train = orig_data.copy()
test = orig_data.copy()
X = train[0:80,0:cols-1]
y = train[0:80,cols-1:cols]
origi_data_train = train[0:80,:]
X_test = test[80:100,0:cols-1]
y_test = test[80:100,cols-1:cols]
origi_data_test = test[80:100,:]
# convert to numpy arrays and initalize the parameter array theta
#X = np.matrix(X.values)
#y = np.matrix(data.iloc[:,3:4].values) #np.array(y.values)
#X.shape, y.shape, theta.shape


#5000次迭代
theta = np.zeros([1, 3])
theta = runExpe(origi_data_train, theta, n, STOP_ITER, thresh=50000, alpha=0.001)
#损失降低到0.000001
theta = np.zeros([1, 3])
theta = runExpe(origi_data_train, theta, n, STOP_COST, thresh=0.000001, alpha=0.001)
#梯度降低到0.05
theta = np.zeros([1, 3])
theta = runExpe(origi_data_train, theta, n, STOP_GRAD, thresh=0.05, alpha=0.001)
#one_batch
runExpe(origi_data_train, theta, 1, STOP_ITER, thresh=5000, alpha=0.001)
#one_batch_small_lr
runExpe(origi_data_train, theta, 1, STOP_ITER, thresh=15000, alpha=0.000002)
#mini_batch
runExpe(origi_data_train, theta, 16, STOP_ITER, thresh=15000, alpha=0.001)


#with_preprocess
scaled_data = origi_data_train.copy()
#scaled_data[:, 1:3] = pp.scale(origi_data_train[:, 1:3])
theta = np.zeros([1, 3])
theta = runExpe(scaled_data, theta, n, STOP_ITER, thresh=50000, alpha=0.001)

theta = np.zeros([1, 3])
theta = runExpe(scaled_data, theta, n, STOP_COST, thresh=0.000001, alpha=0.001)

theta = np.zeros([1, 3])
theta = runExpe(scaled_data, theta, n, STOP_GRAD, thresh=0.02, alpha=0.001)

theta = np.zeros([1, 3])
theta = runExpe(scaled_data, theta, 1, STOP_GRAD, thresh=0.002/5, alpha=0.001)

theta = np.zeros([1, 3])
theta = runExpe(scaled_data, theta, 16, STOP_GRAD, thresh=0.002*2, alpha=0.001)


#f分析正确率（测试集）
scaled_X = origi_data_test[:, :3]
y = origi_data_test[:,3]
predictions = predict(scaled_X, theta)
correct = [1 if ((a == 1 and b == 1) or (a == 0 and b == 0)) else 0 for (a, b) in zip(predictions, y)]
accuracy = float(sum(map(int, correct))) / float(len(correct)) *100
print ('accuracy = {0}%'.format(accuracy))









