import torch

a = torch.empty(2,2)
print(a)

b = torch.rand(5,6)
print(b)

c = torch.zeros(5,3,dtype = torch.long)
print(c)

d = c.new_ones(5,3,dtype = torch.long)
print(d)
print(d.size())

e = c*d
print(e)

f = c+d
print(f)
