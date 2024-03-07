from ortools.linear_solver import pywraplp
import time

A = list(map(int,input().split()))
N=A[0]
K=A[1]
item =[]
for row in range(N):
    row=list(map(int,input().split()))
    item.append(row)
trucks=[]
for row in range(K):
    row =list(map(int,input().split()))
    trucks.append(row)
start_time = time.time()
#W,H and C of trucks
W = [trucks[i][0] for i in range(K)]
H = [trucks[i][1] for i in range(K)]
C = [trucks[i][2] for i in range(K)]


# h,w for each item
h = [item[i][1] for i in range(N)]
w = [item[i][0] for i in range(N)]


n = len(h)  # number of items
m = len(H)  # max number of bins
solver = pywraplp.Solver.CreateSolver('SCIP')
M=10**10

D={}
for i in range(n):
    for k in range(m):
        D[i,k]=solver.IntVar(0, 1, f'D_{i}_{k}')
o = {}  # if Ro = 1 then rotation = 90 degree, else 0
l = []  # left coordination of item
r = []  # right coordination of item
t = []  # top coordination of item
b = []  # bottom coodination of item

z=[solver.IntVar(0,1,f'z_{k}') for k in range(m)]
for i in range(n):
        o[i] = solver.IntVar(0, 1, f'o_{i}')

        # coordinate
        l.append(solver.IntVar(0, max(W),f'l_{i}'))
        r.append(solver.IntVar(0, max(W),f'r_{i}'))
        t.append(solver.IntVar(0, max(H), f't_{i}'))
        b.append(solver.IntVar(0, max(H), f'b_{i}'))

        solver.Add(r[i] == (1-o[i]) * w[i] + o[i] * h[i] + l[i])
        solver.Add(t[i] == o[i] * w[i] +(1- o[i]) * h[i] + b[i])

for k in range(m):
    q = solver.IntVar(0, n, f'q[{k}]')
    solver.Add(q == sum(D[(i, k)] for i in range(n)))
    solver.Add(z[k]<=q*M)
    solver.Add(z[k]*M >= q)

for i in range(n):
    solver.Add((sum(D[i,k] for k in range(m))==1))

for i in range(n - 1):
    for j in range(i + 1, n):
        for k in range(m):
            e = solver.IntVar(0, 1, f'e[{i}][{j}]')
            solver.Add(e >= D[i, k] + D[j, k] - 1)
            solver.Add(e <= D[i, k])
            solver.Add(e <= D[j, k])

            # Binary variables for each constraint
            c1 = solver.IntVar(0, 1, f'c1[{i}][{j}]')
            c2 = solver.IntVar(0, 1, f'c2[{i}][{j}]')
            c3 = solver.IntVar(0, 1, f'c3[{i}][{j}]')
            c4 = solver.IntVar(0, 1, f'c4[{i}][{j}]')

            # Constraints that the binary variables must satisfy
            solver.Add(r[i] <= l[j] + M * (1 - c1))
            solver.Add(r[j] <= l[i] + M * (1 - c2))
            solver.Add(t[i] <= b[j] + M * (1 - c3))
            solver.Add(t[j] <= b[i] + M * (1 - c4))

            solver.Add(c1 + c2 + c3 + c4 + (1 - e) * M >= 1)
            solver.Add(c1 + c2 + c3 + c4 <= e * M)

for i in range(n):
    for k in range(m):
        solver.Add(r[i] <= (1 - D[i, k]) * M + W[k])
        solver.Add(l[i] <= (1 - D[i, k]) * M + W[k])
        solver.Add(t[i] <= (1 - D[i, k]) * M + H[k])
        solver.Add(b[i] <= (1 - D[i, k]) * M + H[k])

cost = sum(z[k]*C[k] for k in range(m))
solver.Minimize(cost)
solver.set_time_limit(300*1000)
status = solver.Solve()
a=[]
for k in range(m):
    if z[k].solution_value()==1:
        a.append(k+1)
    else:
        a.append(0)
print(status)
if status == pywraplp.Solver.OPTIMAL:
    line = 0
    for k in range(m):
        for i in range(n):
            if D[i,k].solution_value() == 1:
                line = line + 1
                print(
                    f"{line}" + " "+f'{solver.Objective().Value()}' +" "+ f'{a[k]}' + " " + f"{int(l[i].solution_value())}" + " " + f'{int(b[i].solution_value())}' + " " + f"{int(o[i].solution_value())}" + " " + f'{w[i]}' + " " + f'{h[i]}')
            else:
                continue
end_time = time.time()

print(end_time - start_time)