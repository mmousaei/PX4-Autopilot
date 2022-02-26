from cgi import test
import imp
import numpy as np
from read_csv import IROS_Csv
import paper_plot

if __name__ == '__main__':
	print("plotting!")
	reader = IROS_Csv()
	reader.read_file('test_data/motor_failure_1/motor_failure_1.csv')
	# paper_plot.plot_control_sp(reader.control_sp)
	# paper_plot.plot_control_al_vs_no_opt(reader.control_sp, reader.control_allocated, reader.control_allocated_no_opt)
	# print("norm_control_sp[0] = ", reader.norm_control_sp[:,0])
	# print("norm_control_sp[0] = ", reader.norm_control_sp[1][0])
	paper_plot.plot_norm_c_sp_vs_norm_c_all(reader.norm_control_sp, reader.norm_control_allocated)

