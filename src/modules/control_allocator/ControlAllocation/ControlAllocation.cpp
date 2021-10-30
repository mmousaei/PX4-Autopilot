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
	const float act_trim_k [NUM_ACTUATORS] = {0.0f,0.0f,0.0f,0.0f,_actuator_trim(4), _actuator_trim(5), _actuator_trim(6), _actuator_trim(7), 0.0f,0.0f,0.0f,0.0f};
	_actuator_trim_known = matrix::Vector<float, NUM_ACTUATORS>(act_trim_k);
	const float act_trim_uk [NUM_ACTUATORS] = {_actuator_trim(0), _actuator_trim(1), _actuator_trim(2), _actuator_trim(3), 0.0f,0.0f,0.0f,0.0f,_actuator_trim(8), _actuator_trim(9), _actuator_trim(10), _actuator_trim(11)};
	_actuator_trim_unknown = matrix::Vector<float, NUM_ACTUATORS>(act_trim_uk);
	const float eff_known[NUM_AXES][NUM_ACTUATORS] = {
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(0, 4), _effectiveness(0, 5), _effectiveness(0, 6), _effectiveness(0, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(1, 4), _effectiveness(1, 5), _effectiveness(1, 6), _effectiveness(1, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(2, 4), _effectiveness(2, 5), _effectiveness(2, 6), _effectiveness(2, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(3, 4), _effectiveness(3, 5), _effectiveness(3, 6), _effectiveness(3, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(4, 4), _effectiveness(4, 5), _effectiveness(4, 6), _effectiveness(4, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f},
										{0.0f,0.0f,0.0f,0.0f,_effectiveness(5, 4), _effectiveness(5, 5), _effectiveness(5, 6), _effectiveness(5, 7),0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}
									};
	_effectiveness_known = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> (eff_known);
	const float eff_unknown[NUM_AXES][NUM_ACTUATORS] = {
										{_effectiveness(0, 0), _effectiveness(0, 1), _effectiveness(0, 2), _effectiveness(0, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(0, 8), _effectiveness(0, 9), _effectiveness(0, 10), _effectiveness(0, 11),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(1, 0), _effectiveness(1, 1), _effectiveness(1, 2), _effectiveness(1, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(1, 8), _effectiveness(1, 9), _effectiveness(1, 10), _effectiveness(1, 11),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(2, 0), _effectiveness(2, 1), _effectiveness(2, 2), _effectiveness(2, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(2, 8), _effectiveness(2, 9), _effectiveness(2, 10), _effectiveness(2, 11),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(3, 0), _effectiveness(3, 1), _effectiveness(3, 2), _effectiveness(3, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(3, 8), _effectiveness(3, 9), _effectiveness(3, 10), _effectiveness(3, 11),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(4, 0), _effectiveness(4, 1), _effectiveness(4, 2), _effectiveness(4, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(4, 8), _effectiveness(4, 9), _effectiveness(4, 10), _effectiveness(4, 11),0.0f,0.0f,0.0f,0.0f},
										{_effectiveness(5, 0), _effectiveness(5, 1), _effectiveness(5, 2), _effectiveness(5, 3),0.0f,0.0f,0.0f,0.0f, _effectiveness(5, 8), _effectiveness(5, 9), _effectiveness(5, 10), _effectiveness(5, 11),0.0f,0.0f,0.0f,0.0f}
									};
	_effectiveness_unknown = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> (eff_unknown);
	
	_control_trim_known = _effectiveness_known * _actuator_trim_known;
	_control_trim_unknown = _effectiveness_unknown * _actuator_trim_unknown;
	
	// printf("_control_trim:\n");
	// _control_trim.print();
	// printf("_control_trim_known:\n");
	// _control_trim_known.print();
	// printf("_control_trim_unknown:\n");
	// _control_trim_unknown.print();

	// printf("_actuator_trim_known:\n");
	// _actuator_trim_known.T().print();
	// printf("_actuator_trim_unknown:\n");
	// _actuator_trim_unknown.T().print();
	// printf("_actuator_trim:\n");
	// _actuator_trim.T().print();

	// printf("_effectiveness_known:\n");
	// _effectiveness_known.print();
	// printf("_effectiveness_unknown:\n");
	// _effectiveness_unknown.print();
	// printf("_effectiveness:\n");
	// _effectiveness.print();

	



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
