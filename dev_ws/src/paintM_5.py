from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

comp_list = [[0 for j in range(0,21)] for i in range(0,6)]
#comp data
base = "../ans_smart/ans"
base_route = base
for j in range(1,21):
    file_route = base_route+"/M_"+str(5)+"_N_"+str(j)+"_sum_writer_0.out"
    f = open(file_route)
    lines = f.readlines()
    sum = 0.0
    for line in lines:
        sum += float(line)
    sum /= len(lines)
    comp_list[1][j] = sum

base = "../ans_mail/ans"
base_route = base
for j in range(1,21):
    file_route = base_route+"/M_"+str(5)+"_N_"+str(j)+"_sum_writer_0.out"
    f = open(file_route)
    lines = f.readlines()
    sum = 0.0
    for line in lines:
        sum += float(line)
    sum /= len(lines)
    comp_list[3][j] = sum

base = "../ans/ans"
base_route = base
for j in range(1,21):
    file_route = base_route+"/M_"+str(5)+"_N_"+str(j)+"_sum_writer_0.out"
    f = open(file_route)
    lines = f.readlines()
    sum = 0.0
    for line in lines:
        sum += float(line)
    sum /= len(lines)
    comp_list[5][j] = sum

#paint
x = np.array([i for i in range(1,21)])
x_new = np.linspace(x.min(),x.max(),300)
comp_1 = np.array(comp_list[1][1:])
comp_1_smo = make_interp_spline(x,comp_1)(x_new)
comp_3 = np.array(comp_list[3][1:])
comp_3_smo = make_interp_spline(x,comp_3)(x_new)
comp_5 = np.array(comp_list[5][1:])
comp_5_smo = make_interp_spline(x,comp_5)(x_new)

plt.title("Constant M and Variable N")
plt.plot(x_new,comp_1_smo,color='springgreen',label='smart_proj:M=1')
plt.plot(x_new,comp_3_smo,color='royalblue',label='mail_proj:M=3')
plt.plot(x_new,comp_5_smo,color='yellow',label='ours_proj:M=5')
plt.legend()

plt.xlabel('N')
plt.ylabel('Time consuming for a single pub/s')
plt.savefig("./const_M_5.png")
plt.show()