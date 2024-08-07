from sympy import *

vm = symbols('vm', real=True) # Battery voltage
rb = symbols("V, rb", real=True) # 

n = Symbol('n')
i = Symbol('i')
v = IndexedBase('v', real=True)
e = IndexedBase('e', real=True)
r = IndexedBase('r', real=True)
b = IndexedBase('b', real=True)

C = Function('C')
def mx(a, E):
    return E*a + (1 - E)*vm

print(solve(12 - rb * sum([(mx(v[i], e[i]) + b[i]) / r[int(i/4)] for i in range(8)]) - vm, vm)[0])
