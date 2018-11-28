import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

size = 1000
count = 501
var = 3000
e = 0.97
move = 30
pos = 100
m = np.ones((size, size))

plt.ion()
plt.imshow(m, cmap='hot', interpolation='nearest')
plt.show()

def update():
    global pos, size, var, e, move, m
    a = multivariate_normal.pdf(np.linspace(0, size, num=size), mean=pos, cov=var)
    b = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
    c = b / b.sum()
    m = (m * e) + c
    pos = pos + move
    plt.imshow(m, cmap='hot', interpolation='nearest')
    plt.draw()

while True:
    input("Press...")
    update()
    plt.imshow(m, cmap='hot', interpolation='nearest')
    plt.show()
    pos = pos + move
    input("Waiting...")
