import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

count = 501
var = 3000
a = multivariate_normal.pdf(np.linspace(0, count - 1, num=count), mean=(count - 1) / 2, cov=var)
b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
print("-")
print(b.sum())
print("-")
c = b / b.sum()
print("\n\n")
print(a)
print("\n\n")
print(b)
print("\n\n")
print(c)

size = 1001
e = 0.9
move = 30
pos = 100

m = np.ones((size, size))

m = m / (m.sum() * e)

while True:

    m = m * e

    for i in range(0, count):
        for j in range(0, count):
            m[pos + i][pos + j] = min(m[pos+i][pos+j] + c[i][j] * e, 1)

    plt.imshow(m, cmap='hot', interpolation='nearest')
    plt.show()
    pos += move
    input("Waiting...")
