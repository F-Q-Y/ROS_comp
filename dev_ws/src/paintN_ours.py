from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

#M sub_question
#input
#my data
base = "../ans/ans"
my_list = [[0 for j in range(0,13)] for i in range(0,6)]
for i in [1,3,5]:
    base_route = base
    for j in range(1,13):
        file_route = base_route+"/M_"+str(j)+"_N_"+str(i)+"_sum_writer_0.out"
        f = open(file_route)
        lines = f.readlines()
        sum = 0.0
        for line in lines:
            sum += float(line)
        sum /= len(lines)
        my_list[i][j] = sum

#paint
x = np.array([i for i in range(1,13)])
x_new = np.linspace(x.min(),x.max(),300)
my_1 = np.array(my_list[1][1:])
my_1_smo = make_interp_spline(x,my_1)(x_new)
my_3 = np.array(my_list[3][1:])
my_3_smo = make_interp_spline(x,my_3)(x_new)
my_5 = np.array(my_list[5][1:])
my_5_smo = make_interp_spline(x,my_5)(x_new)


plt.title("Constant N and Variable M")
plt.plot(x_new,my_1_smo,color='green',label='our_proj:N=1')
plt.plot(x_new,my_3_smo,color='red',label='our_proj:N=3')
plt.plot(x_new,my_5_smo,color='blue',label='our_proj:N=5')
plt.legend()

plt.xlabel('N')
plt.ylabel('Time consuming for a single pub/s')
plt.savefig("./const_N_ours.png")
plt.show()