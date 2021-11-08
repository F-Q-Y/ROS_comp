from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

#comp data
base = "../ans_comp/ans"
comp_list = [[0 for j in range(0,13)] for i in range(0,6)]
for i in [1,3,5]:
    base_route = base
    for j in range(1,13):
        file_route = base_route+"/M_"+str(j)+"_N_"+str(i)+"_sum_writer.out"
        f = open(file_route)
        lines = f.readlines()
        sum = 0.0
        for line in lines:
            sum += float(line)
        sum /= len(lines)
        comp_list[i][j] = sum

#paint
x = np.array([i for i in range(1,13)])
x_new = np.linspace(x.min(),x.max(),300)
comp_1 = np.array(comp_list[1][1:])
comp_1_smo = make_interp_spline(x,comp_1)(x_new)
comp_3 = np.array(comp_list[3][1:])
comp_3_smo = make_interp_spline(x,comp_3)(x_new)
comp_5 = np.array(comp_list[5][1:])
comp_5_smo = make_interp_spline(x,comp_5)(x_new)

plt.title("Constant N and Variable M")
plt.plot(x_new,comp_1_smo,color='springgreen',label='comp_proj:N=1')
plt.plot(x_new,comp_3_smo,color='royalblue',label='comp_proj:N=3')
plt.plot(x_new,comp_5_smo,color='yellow',label='comp_proj:N=5')
plt.legend()

plt.xlabel('M')
plt.ylabel('Time consuming for a single pub/s')
plt.savefig("./const_N_comp.png")
plt.show()