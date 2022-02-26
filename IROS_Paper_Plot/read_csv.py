import csv

from dbus import SessionBus

class IROS_Csv:

	def __init__(self):
		self.control_sp = []
		self.control_allocated = []
		self.control_allocated_no_opt = []
		self.norm_control_sp = []
		self.norm_control_allocated = []
		self.norm_control_allocated_no_opt = []
		self.actuator_sp = []
		self.actuator_sp_no_opt = []
		self.airspeed = []
		self.roll = []
		self.pitch = []
		self.yaw = []
		self.fail_id = []
		self.fail_val = []
	def print_file(self):
		for row in self.file:
			print(', '.join(row))
	def read_file(self, filename):
		with open(filename) as csv_file:
			csv_reader = csv.reader(csv_file, delimiter=',')
			line_count = 0
			for row in csv_reader:
				if line_count == 0:
					print(f'Data column names are {", ".join(row)}')
					line_count += 1
				else:
					# print(row)
					self.control_sp.append([row[0], row[1], row[2], row[3], row[4], row[5]])
					self.actuator_sp.append([row[6], row[7], row[8], row[9], row[10], row[11], row[12], row[13], row[14], row[15], row[16], row[17]])
					self.airspeed.append([row[18]])
					self.roll.append([row[19]])
					self.pitch.append([row[20]])
					self.control_allocated.append([row[21], row[22], row[23], row[24], row[25], row[26]])
					self.control_allocated_no_opt.append([row[27], row[28], row[29], row[30], row[31], row[32]])
					self.norm_control_sp.append([row[33]])
					self.norm_control_allocated.append([row[34]])
					self.norm_control_allocated_no_opt.append([row[35]])
					self.actuator_sp_no_opt.append([row[36], row[37], row[38], row[39], row[40], row[41], row[42], row[43], row[44], row[45], row[46], row[47]])
					self.yaw.append([row[48]])
					self.fail_id.append([row[49]])
					self.fail_val.append([row[50]])

				line_count += 1
			print(f'Processed {line_count} lines.')
