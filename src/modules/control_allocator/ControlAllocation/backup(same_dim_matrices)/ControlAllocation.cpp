/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocation.cpp
 *
 * Interface for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocation.hpp"

void
ControlAllocation::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_trim, int num_actuators)
{
	_effectiveness = effectiveness;
	_actuator_trim = actuator_trim;
	clipActuatorSetpoint(_actuator_trim);
	_control_trim = _effectiveness * _actuator_trim;
	// ADDED
	_actuator_trim_known = matrix::Vector<float, NUM_ACTUATORS>(_actuator_trim);
	for(int i = 0; i < NUM_ACTUATORS; i++) {
		if ( i < 4 || i > 7) {
			_actuator_trim_known(i) = 0.0f;
		}
	}
	_actuator_trim_unknown = matrix::Vector<float, NUM_ACTUATORS>(_actuator_trim);
	for(int i = 0; i < NUM_ACTUATORS; i++) {
		if ( i < 8 && i > 3) {
			_actuator_trim_unknown(i) = 0.0f;
		}
	}

	const float eff_known[NUM_AXES][NUM_ACTUATORS] = {
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 0), _effectiveness(5, 0), _effectiveness(6, 0), _effectiveness(7, 0),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 1), _effectiveness(5, 1), _effectiveness(6, 1), _effectiveness(7, 1),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 2), _effectiveness(5, 2), _effectiveness(6, 2), _effectiveness(7, 2),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 3), _effectiveness(5, 3), _effectiveness(6, 3), _effectiveness(7, 3),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 4), _effectiveness(5, 4), _effectiveness(6, 4), _effectiveness(7, 4),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 5), _effectiveness(5, 5), _effectiveness(6, 5), _effectiveness(7, 5),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}
									};
	_effectiveness_known = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> (eff_known);
	const float eff_unknown[NUM_AXES][NUM_ACTUATORS] = {
										{_effectiveness(0, 0), _effectiveness(1, 0), _effectiveness(2, 0), _effectiveness(3, 0),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 0), _effectiveness(9, 0), _effectiveness(10, 0), _effectiveness(11, 0),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(0, 1), _effectiveness(1, 1), _effectiveness(2, 1), _effectiveness(3, 1),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 1), _effectiveness(9, 1), _effectiveness(10, 1), _effectiveness(11, 1),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(0, 2), _effectiveness(1, 2), _effectiveness(2, 2), _effectiveness(3, 2),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 2), _effectiveness(9, 2), _effectiveness(10, 2), _effectiveness(11, 2),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(0, 3), _effectiveness(1, 3), _effectiveness(2, 3), _effectiveness(3, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 3), _effectiveness(9, 3), _effectiveness(10, 3), _effectiveness(11, 3),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(0, 4), _effectiveness(1, 4), _effectiveness(2, 4), _effectiveness(3, 4),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 4), _effectiveness(9, 4), _effectiveness(10, 4), _effectiveness(11, 4),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(0, 5), _effectiveness(1, 5), _effectiveness(2, 5), _effectiveness(3, 5),0.0f,0.0f,0.0f,0.0f, _effectiveness(8, 5), _effectiveness(9, 5), _effectiveness(10, 5), _effectiveness(11, 5),0.0f,0.0f,0.0f,0.0f}
									};
	_effectiveness_unknown = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> (eff_unknown);
	
	_control_trim_known = _effectiveness_known * _actuator_trim_known;
	_control_trim_unknown = _effectiveness_unknown * _actuator_trim_unknown;
	// for(int i = 0; i < NUM_KNOWN; ++i) {
	// 	printf("_actuator_trim_known(%d) = %.3f\n", i , double(_actuator_trim_known(i)));
	// }
	// for(int i = 0; i < NUM_UNKNOWN; ++i) {
	// 	printf("_actuator_trim_unknown(%d) = %.3f\n", i , double(_actuator_trim_unknown(i)));
	// }
	// printf("effectiveness known = \n");
	
	// 		printf("%.3f, ", double(_effectiveness_known(0, 0)));
	// 		printf("%.3f, ", double(_effectiveness_known(0, 1)));
	// 		printf("%.3f, ", double(_effectiveness_known(0, 2)));
	// 		printf("%.3f, ", double(_effectiveness_known(0, 3)));
	// 		printf("%.3f, ", double(_effectiveness_known(0, 4)));
	// 		printf("%.3f, ", double(_effectiveness_known(0, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_known(1, 0)));
	// 		printf("%.3f, ", double(_effectiveness_known(1, 1)));
	// 		printf("%.3f, ", double(_effectiveness_known(1, 2)));
	// 		printf("%.3f, ", double(_effectiveness_known(1, 3)));
	// 		printf("%.3f, ", double(_effectiveness_known(1, 4)));
	// 		printf("%.3f, ", double(_effectiveness_known(1, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_known(2, 0)));
	// 		printf("%.3f, ", double(_effectiveness_known(2, 1)));
	// 		printf("%.3f, ", double(_effectiveness_known(2, 2)));
	// 		printf("%.3f, ", double(_effectiveness_known(2, 3)));
	// 		printf("%.3f, ", double(_effectiveness_known(2, 4)));
	// 		printf("%.3f, ", double(_effectiveness_known(2, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_known(3, 0)));
	// 		printf("%.3f, ", double(_effectiveness_known(3, 1)));
	// 		printf("%.3f, ", double(_effectiveness_known(3, 2)));
	// 		printf("%.3f, ", double(_effectiveness_known(3, 3)));
	// 		printf("%.3f, ", double(_effectiveness_known(3, 4)));
	// 		printf("%.3f, ", double(_effectiveness_known(3, 5)));
	// 		printf("\n");
	// printf("\n");

	// printf("effectiveness unknown = \n");
	
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(0, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(1, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(2, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(3, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(4, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(5, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(6, 5)));
	// 		printf("\n");
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 0)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 1)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 2)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 3)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 4)));
	// 		printf("%.3f, ", double(_effectiveness_unknown(7, 5)));
	// 		printf("\n");
			
	// printf("\n");



	// ADDED
	_num_actuators = num_actuators;

	// make sure unused actuators are initialized to trim
	for (int i = num_actuators; i < NUM_ACTUATORS; ++i) {
		_actuator_sp(i) = _actuator_trim(i);
	}
}

void
ControlAllocation::setActuatorSetpoint(
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_sp)
{
	// Set actuator setpoint
	_actuator_sp = actuator_sp;

	// Clip
	clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated = _effectiveness * _actuator_sp;

}

void
ControlAllocation::clipActuatorSetpoint(matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator) const
{
	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_max(i) < _actuator_min(i)) {
			actuator(i) = _actuator_trim(i);

		} else if (actuator(i) < _actuator_min(i)) {
			actuator(i) = _actuator_min(i);

		} else if (actuator(i) > _actuator_max(i)) {
			actuator(i) = _actuator_max(i);
		}
	}
}

matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::normalizeActuatorSetpoint(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator)
const
{
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_normalized;

	for (size_t i = 0; i < ControlAllocation::NUM_ACTUATORS; i++) {
		if (_actuator_min(i) < _actuator_max(i)) {
			actuator_normalized(i) = -1.0f + 2.0f * (actuator(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));

		} else {
			actuator_normalized(i) = -1.0f + 2.0f * (_actuator_trim(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));
		}
	}

	return actuator_normalized;
}
