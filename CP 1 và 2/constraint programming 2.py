from ortools.sat.python import cp_model
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
start_time=time.time()

#W,H and C of trucks
W = [trucks[i][0] for i in range(K)]
H = [trucks[i][1] for i in range(K)]
C = [trucks[i][2] for i in range(K)]


# h,w for each item
h = [item[i][1] for i in range(N)]
w = [item[i][0] for i in range(N)]


n = len(h)  # number of items
m = len(H)  # max number of bins

model=cp_model.CpModel()

#decision variable
D={}

#rotation variable
o=[]

for i in range(n):
    o.append(model.NewBoolVar(f'o_{i}'))
    for k in range(m):
        D[i,k] = model.NewBoolVar(f'D_{i}_{k}')
#bin has been used
z=[model.NewBoolVar(f'z_{k}') for k in range(m)]

r = []  # right coordinate
l = []  # left coordinate
t = []  # top coordinate
b = []  # bottom coordinate
w_new=[]
h_new=[]
for i in range(n):
    dim1=w[i]
    dim2=h[i]

    l.append(model.NewIntVar(0, max(W), f'l_{i}'))

    b.append(model.NewIntVar(0, max(H), f'b_{i}'))

    w_new.append(model.NewIntVar(0,max(W),f'w_new{i}'))
    h_new.append(model.NewIntVar(0,max(H),f'h_new{i}'))

    model.Add(w_new[i]==w[i]).OnlyEnforceIf(o[i].Not())
    model.Add(w_new[i]==h[i]).OnlyEnforceIf(o[i])
    model.Add(h_new[i]==h[i]).OnlyEnforceIf(o[i].Not())
    model.Add(h_new[i]== w[i]).OnlyEnforceIf(o[i])

for i in range(n):
        model.Add(sum(D[i, k] for k in range(m)) == 1)
for i in range(n):
        for k in range(m):
            model.Add(l[i]+w_new[i] <= W[k]).OnlyEnforceIf(D[i, k])
            model.Add(b[i]+h_new[i] <= H[k]).OnlyEnforceIf(D[i, k])



for i in range(n):
        for j in range(i+1, n):
            a1 = model.NewBoolVar('a1')
            model.Add(l[i]<= l[j]-w_new[i]).OnlyEnforceIf(a1)
            model.Add(l[i] > l[j]-w_new[i]).OnlyEnforceIf(a1.Not())
            a2 = model.NewBoolVar('a2')
            model.Add(b[i]+h_new[i] <= b[j]).OnlyEnforceIf(a2)
            model.Add(b[i]+h_new[i] > b[j]).OnlyEnforceIf(a2.Not())
            a3 = model.NewBoolVar('a3')
            model.Add(l[j]<=l[i]-w_new[j]).OnlyEnforceIf(a3)
            model.Add(l[j] > l[i]-w_new[j]).OnlyEnforceIf(a3.Not())
            a4 = model.NewBoolVar('a4')
            model.Add(b[j] <= b[i]-h_new[j]).OnlyEnforceIf(a4)
            model.Add(b[j] > b[i]-h_new[j]).OnlyEnforceIf(a4.Not())

            for k in range(m):
                model.AddBoolOr(a1, a2, a3, a4).OnlyEnforceIf(D[i, k], D[j, k])

for k in range(m):
    b1=model.NewBoolVar("b")
    model.Add(sum(D[i, k] for i in range(n)) == 0).OnlyEnforceIf(b1)
    model.Add(z[k] == 0).OnlyEnforceIf(b1)
    model.Add(sum(D[i, k] for i in range(n)) >=1).OnlyEnforceIf(b1.Not())
    model.Add(z[k] == 1).OnlyEnforceIf(b1.Not())

cost=sum(z[k]*C[k] for k in range(m))
model.Minimize(cost)

solver = cp_model.CpSolver()
solver.parameters.max_time_in_seconds=300
status = solver.Solve(model)
f=[]
for k in range(m):
    if solver.Value(z[k])==1:
        f.append(k+1)
    else:
        f.append(0)
if status==cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    line=0
    for k in range(m):
            for i in range(n):
                if solver.Value(D[i,k])==1:
                    line=line+1
                    print( f"{line}" + " "+f'{solver.ObjectiveValue()}' + " " +f'{f[k]}' + " " + f"{solver.Value(l[i])}" + " " + f'{solver.Value(b[i])}' + " " + f"{solver.Value(o[i])}" + " " + f'{w[i]}' + " " + f'{h[i]}')
                else:
                     continue
end_time = time.time()

print(end_time-start_time)