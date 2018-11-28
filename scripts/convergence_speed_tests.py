import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
import time

count = 501
var = 3000
size = 1001
e = 0.9
move = 30
pos = 100
num_trials = 100

def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.2f ms', method.__name__, (te - ts) * 1000)
        return result

    return timed

@timeit
def multi_var():
    x = np.linspace(0, size, num=size)
    y = np.linspace(0, size, num=size)
    X, Y = np.meshgrid(x, y)
    p = np.dstack((X, Y))
    m = np.ones((size, size))
    for x in range(num_trials):
        a = multivariate_normal([pos, pos], [[var,0],[0, var]])
        m = (m * e) + a.pdf(p)
    #plt.imshow(a.pdf(p), cmap='hot', interpolation='nearest')
    #plt.show()

@timeit
def test_iter():
    a = multivariate_normal.pdf(np.linspace(0, count - 1, num=count), mean=(count - 1) / 2, cov=var)
    b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
    c = b / b.sum()    
    m = np.ones((size, size))
    
    for n in range(num_trials):
    
        m = m * e
    
        for i in range(0, count):
            for j in range(0, count):
                m[pos + i][pos + j] = min(m[pos+i][pos+j] + c[i][j] * e, 1)
    
    #plt.imshow(m, cmap='hot', interpolation='nearest')
    #plt.show()
    
@timeit
def test_block():

    for i in range(num_trials):
        a = multivariate_normal.pdf(np.linspace(0, size, num=size), mean=pos, cov=var)
        b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
        c = b / b.sum()    
        m = np.ones((size, size))
        
        m = (m * e) + c
    
    #plt.imshow(m, cmap='hot', interpolation='nearest')
    #plt.show()

if __name__ == '__main__':
    multi_var()
    test_iter()
    test_block()
