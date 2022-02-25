from cgi import test
import imp
import numpy as np
from read_csv import IROS_Csv
import paper_plot

if __name__ == '__main__':
	print("plotting!")
	reader = IROS_Csv()
	reader.read_file('/home/mohammad/workspace/px4_tilt_test/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/test.csv')
	# paper_plot.plot_control_sp(reader.control_sp)
	paper_plot.plot_control_al_vs_no_opt(reader.control_sp, reader.control_allocated, reader.control_allocated_no_opt)

