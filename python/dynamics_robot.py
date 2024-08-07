# Outputs a c file containing the physics math. 
# To use, run this file, and copy the contents of the inside of "func" 
# inside output.c to the "jacobian" function in SwerveSim.java. Additionally,
# the function fix() must be wrapped around variables with negative powers.
# The header file can be discarded.
# This file may take several seconds to run.

from sympy import *
from sympy.physics.mechanics import *
from sympy.printing.latex import LatexPrinter
from sympy.core.function import UndefinedFunction
from c import ListSymbol

wr = symbols('wr', positive=True) # Wheel radius
V = symbols('V:8', real=True) # Motor voltages [turning, driving]
mr, mw = symbols('mr, mw', positive=True) # Robot, wheel mass
It, Ir, IR = symbols('It, Ir, IR', positive=True) # Wheel turning, rolling, robot inertia
a, b = symbols('a, b', positive=True) # Distance from center to front, side
k = symbols("k", positive=True) # Friction smoothing
stuck = ListSymbol('stuck', 4, 1) # 0 means wheel cannot rotate, 1 means no difference
kVD, kAD = symbols("kVD, kAD", positive=True) # Motor constants, motor-wheel gear ratio of drive motor
kVT, kAT = symbols("kVT, kAT", positive=True) # Motor constants, motor-wheel gear ratio of turning motor

delta = dynamicsymbols('delta:4', real=True) # Turning angles
phi = dynamicsymbols('phi:4', real=True) # Rolling angles
x, y, theta = dynamicsymbols('x, y, theta', real=True)

Fz = (mr + 4 * mw) * 9.80665

N = ReferenceFrame('N')

O = Point('O')
O.set_vel(N, 0)

Cw = O.locatenew('Cw', x * N.x + y * N.y)
Cw.set_vel(N, x.diff()*N.x + y.diff()*N.y)

q = Matrix([x, y, theta, *phi, *delta])
dq = q.diff()

# Robot reference frame
R = N.orientnew('R', 'Axis', [theta, N.z])

# Wheel hub centers
H = [Cw.locatenew(f'H{i}', xx * R.x + yy * R.y) for (i, xx, yy) in [(0, a, b), (1, -a, b), (2, -a, -b), (3, a, -b)]]

# Set velocities
for h in H:
    h.v2pt_theory(Cw, N, R)

# Turning and rolling reference frames
Wt = [R.orientnew(f'Wt{i}', 'Axis', [delta[i], R.z]) for i in range(4)]
Wr = [Wt[i].orientnew(f'Wr{i}', 'Axis', [phi[i], Wt[i].y]) for i in range(4)]

# Wheel inertias
Iw = [inertia(Wt[i], It, Ir, It) for i in range(4)]

# Wheel rigidbodies
wheels = [RigidBody(f'wheel{i}', H[i], Wr[i], mw, (Iw[i], H[i])) for i in range(4)]

# Body inertia
Ib = inertia(R, 0, 0, IR)

body = RigidBody('body', Cw, R, mr, (Ib, Cw))

# Contanct points
C = [H[i].locatenew(f'C{i}', -wr*R.z) for i in range(4)]

for i in range(4):
    C[i].v2pt_theory(H[i], N, Wr[i])

# direction = atan2(y.diff(t), x.diff(t))
# slip = [180 / pi * (direction - (phi[i] + theta)) for i in range(4)]

# print(Abs(x).diff())
# exit()

deriv = Function('deriv')
fix = Function('fix')

# Extemely cursed
class smooth(Function):
    def fdiff(self, i):
        return deriv(Symbol('this::smooth'), self.args[0])

class cof(Function):
    def fdiff(self, i):
        return deriv(Symbol('this::cof'), self.args[0])

class rollingCof(Function):
    def fdiff(self, i):
        return deriv(Symbol('this::rollingCof'), self.args[0])

class copySign(Function):
    def fdiff(self, i):
        if i == 2:
            return 0

        return (self.args[0] * sign(self.args[1])).diff()


# x, y = symbols('x, y')
# print(copySign(x, y).diff(y))
# exit()

# Friction and torques and stuff
Vdiff = [-C[i].vel(N).express(N) for i in range(4)]
F = [cof(Vdiff[i].magnitude()) * Vdiff[i].normalize() * Fz / 4 * smooth(Vdiff[i].magnitude()) for i in range(4)]
tau_drive = [kVD * phi[i].diff() + kAD * V[i] for i in range(4)]
tau_turn = [kVT * delta[i].diff() + kAT * V[i+4] for i in range(4)]

tau_friction = [copySign(rollingCof(phi[i].diff()), phi[i].diff()) * smooth(phi[i].diff()) for i in range(4)]

fl = [sf for i in range(4) for sf in [(Wr[i], Wt[i].y * stuck[i] * (tau_drive[i] - wr * F[i].dot(Wt[i].x) - tau_friction[i])), (Wt[i], R.z * (tau_turn[i] + theta.diff().diff() * It))]] + [(H[i], F[i]) for i in range(4)]

# Vdiff = [-C[i].vel(N).express(N) for i in range(4)]
# F = [1.5 * Vdiff[i].normalize() * Fz / 4 * smooth(Vdiff[i].magnitude()) for i in range(4)]
# tau_drive = [kVD * phi[i].diff() + kAD * V[i] for i in range(4)]
# tau_turn = [kVT * delta[i].diff() + kAT * V[i+4] for i in range(4)]

# tau_friction = [copySign(1.25, phi[i].diff()) for i in range(4)]

# fl = [sf for i in range(4) for sf in [(Wr[i], Wt[i].y * stuck[i] * (tau_drive[i] - wr * F[i].dot(Wt[i].x) - tau_friction[i])), (Wt[i], R.z * (tau_turn[i] + theta.diff().diff() * It))]] + [(H[i], F[i]) for i in range(4)]

n = len(q)
qs = symbols(f'q:{n}', real=True)
dqs = symbols(f'dq:{n}', real=True)
dt = Symbol('dt', positive=True)

def rep(x):
    return x.xreplace({dq[i]: dqs[i] for i in range(n)}).xreplace({q[i]: qs[i] for i in range(n)})

Lag = Lagrangian(N, body, *wheels) 
lm = LagrangesMethod(Lag, q, forcelist=fl, frame=N)
le = lm.form_lagranges_equations()
rhs = rep(lm.mass_matrix_full).LUsolve(rep(lm.forcing_full))

q1 = symbols(f'Q:{2*n}', real=True)
rhs2 = rhs.xreplace({dqs[i]: q1[i+n] for i in range(n)}).xreplace({qs[i]: q1[i] for i in range(n)})
Q = Matrix(qs).col_join(Matrix(dqs))
F = Q - Matrix(q1) - dt * rhs

# Calculate jacobian
# Fj = F.jacobian(Q)

# lvars, [rest] = cse(F, order='none', optimizations='basic')
# print(solve(F, Q))
# exit()

# S = [i / 10 + 0.1 for i in range(22)]
# vsubs = {wr: 0.03683, mr: 60, mw: 0.1, It: 0.01, Ir: 0.01, IR: 5, a: 0.27305, b: 0.27305, stuck: Matrix([1, 1, 1, 1]), kVT: -1.78911236, kVD: -0.1104984419,
#         kAD: 1.413, kAT: 3.75228333, dt: 0.02} | {qs[i]: S[i] for i in range(11)} | {dqs[i]: S[i + 11] for i in range(11)} | {DiracDelta(dqs[i]): 0 for i in range(11)}
# mprint(Fj.xreplace(vsubs))
# lvars, [rest] = cse(Fj, order='none', optimizations='basic')

# print(lvars)
# print(rest)

# for i, sub in enumerate(lvars):
#     lvars[i] = (sub[0], sub[1].xreplace(vsubs))

#     for j in range(i + 1, len(lvars)):
#         lvars[j] = (lvars[j][0], lvars[j][1].subs(lvars[i][0], lvars[i][1]))

#     rest = rest.subs(lvars[i][0], lvars[i][1])

#     if i == 10:
#         print(lvars[i])

# rest = rest.xreplace(vsubs)
# print(rest.doit())
# exit()

# print(cse(hessian(F[11], q1)))
def postprocess(s, e):
    n = Wild('n')
    d = Wild('d', properties=[lambda x: isinstance(x, Symbol)])

    p = Wild('p', properties=[lambda x: x.is_negative])

    for i, x in enumerate(s):
        f = x[1].match(n / d)
        
        if f is not None and f[d].is_positive is None:
            print(f[n], f[d])
            s[i] = (x[0], x[1].xreplace({f[n] / f[d]: fix(f[n] / f[d])}))

        # f2 = s[i][1].match(n ** p)

        # if f2 is not None:
        #     print(f2[n], f2[p])
        #     s[i] = (s[i][0], s[i][1].xreplace({f2[n] ** f2[p]: fix(f2[n] ** f2[p])}))

        

    # def removedirac(x):
    #     # print(x)
    #     return x.xreplace({DiracDelta(w): 0})
    #     # f = x.match(DiracDelta(q), old=True)
    #     # if f is None:
    #     #     return x
    #     # else:
    #     #     print("replacing")
    #     #     print(f)
    #     #     return x.xreplace(f[q], 0)

    # e = [e[0].applyfunc(removedirac)]

    return (s, e)

from codegen import JavaCodeGen

cgen = JavaCodeGen(cse=True, cse_args={'symbols': numbered_symbols(real=True), 'order': 'none', 'optimizations': 'basic', 'postprocess': postprocess})
# Using rhs writes dqdt, Fj writes the jacobian
routine = cgen.routine('func', rhs, global_vars=(wr, mr, mw, It, Ir, IR, a, b, k, stuck, kVD, kAD, kVT, kAT))
cgen.write([routine], "./python/output", True)