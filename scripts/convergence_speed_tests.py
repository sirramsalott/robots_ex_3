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
def multi_var(a):
    x = np.linspace(0, a, num=a)
    y = np.linspace(0, a, num=a)
    X, Y = np.meshgrid(x, y)
    p = np.dstack((X, Y))
    m = np.ones((a, a))
    for x in range(num_trials):
        ar = multivariate_normal([int(a/2), int(a/2)], [[var,0],[0, var]])
        m = (m * e) + ar.pdf(p)

@timeit
def test_iter(x):
    a = multivariate_normal.pdf(np.linspace(0, x - 1, num=x), mean=(x - 1) / 2, cov=var)
    b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
    m = np.ones((size, size))
    for n in range(num_trials):
        m = m * e
        for i in range(0, x):
            for j in range(0, x):
                m[i][j] = min(m[i][j] + b[i][j] * e, 1)
    
@timeit
def test_block(x):
    m = np.ones((x, x))
    for i in range(num_trials):
        a = multivariate_normal.pdf(np.linspace(0, x, num=x), mean=int(x/2), cov=var)
        b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
        m = (m * e) + b

if __name__ == '__main__':
    times = []
    t = range(10, 15)
    for i in t:
        s = time.time()    
        test_iter(int(i))
        e = time.time()
        a = e - s
        s = time.time()
        multi_var(int(i))
        e = time.time()
        b = e - s
        s = time.time()
        test_block(int(i))
        e = time.time()
        c = e - s
        times.append([a, b, c])
    labels = plt.plot(t, times)
    plt.legend(labels, ['1', '2', '3'])
    plt.show()
