from cProfile import label
from operator import le
from joblib import PrintTime
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.font_manager

def test_plot():
	sns.set_context("paper")
	sns.set(font="Verdana")
	sns.set_style({'font.family':'serif', 'font.serif':'Times New Roman'})
	x = np.linspace(0, 14, 100)
	for i in range(1, 5):
		with sns.axes_style("whitegrid"):
			plt.plot(x, np.sin(x + i * .5) * (5 - i))
	plt.title("Test Plot", size=12,fontweight="bold")
	plt.legend(["first", "second", "third", "forth", "fifth"], prop={'size':12}, loc=1)
	plt.show()

def plot_control_sp(control_sp):
	sz = len(control_sp[0])
	c = np.array(control_sp)
	print("contol = ",c[:,1])
	sns.set_context("paper")
	# sns.set(font="Verdana")
	t = np.linspace(0, len(control_sp), len(control_sp))
	sns.set_style({'font.family':'serif', 'font.serif':'Times New Roman'})
	for i in range(sz):
		with sns.axes_style("whitegrid"):
			plt.plot(t, c[:, i])
	plt.title("$Test$ $Plot$", size=12,fontweight="bold")
	plt.legend(["$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$"], prop={'size':12}, loc=1)
	plt.show()

def plot_control_al_vs_no_opt(control_sp, control_al, control_al_no):
	sz = len(control_sp[0])
	c = np.array(control_sp)
	c_al = np.array(control_al)
	c_al_no = np.array(control_al_no)
	print("contol = ",c[:,1])
	sns.set_context("paper")
	# sns.set(font="Verdana")
	t = np.linspace(1, len(control_sp), len(control_sp))
	sns.set_style({'font.family':'serif', 'font.serif':'Times New Roman'})
	fig, ax = plt.subplots()
	for i in range(1):
		with sns.axes_style("whitegrid"):
			ax.plot(c[:, i], label="$C_{"+"sp}"+"^{}$".format(i))
			ax.plot(c_al[:, i], label="$C_{"+"alloc}"+"^{}$".format(i))
			ax.plot(c_al_no[:, i], label="$C_{"+"no opt}"+"^{}$".format(i))
			ax.fill_between(c[:, i], c_al[:, i], c_al_no[:, i], alpha=0.2)
			
	plt.title("$Test$ $Plot$", size=12,fontweight="bold")
	plt.gca().legend(loc='best', ncol=2)
	# plt.legend(["$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$", "$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$", "$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$"], prop={'size':12}, loc=1)
	plt.show()

def plot_norm_c_sp_vs_norm_c_all(norm_c_sp, norm_c_al):
	sz = len(norm_c_sp[0])
	c = np.array(norm_c_sp)
	c_al = np.array(norm_c_al)
	# sns.set_context("paper")
	# sns.set(font="Verdana")
	t = np.linspace(1, len(norm_c_sp), len(norm_c_sp))
	# sns.set_style({'font.family':'serif', 'font.serif':'Times New Roman'})
	print(c_al[:, 0])
	
	# with sns.axes_style("whitegrid"):
	d = 50
	plt.plot(c[520-d:520+d, 0], label="$||C_{"+"sp}||_2^2")
	# plt.plot(c_al[520-d:520+d, 0], label="$||C_{"+"alloc}||_2^2")

	plt.ylim(-10,15)
	ax = plt.title("$Test$ $Plot$", size=12,fontweight="bold")
	# # plt.gca().legend(loc='best', ncol=2)
	# # plt.legend(["$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$", "$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$", "$C_{sp}^0$", "$C_{sp}^1$", "$C_{sp}^2$", "$C_{sp}^3$", "$C_{sp}^4$", "$C_{sp}^5$"], prop={'size':12}, loc=1)
	
	plt.show()
