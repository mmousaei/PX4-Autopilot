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
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationPseudoInverse.hpp"

void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_trim, int num_actuators)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, num_actuators);
	_mix_update_needed = true;
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		_mix = matrix::geninv(_effectiveness);
		_mix_unknown = matrix::geninv(_effectiveness_unknown);
		_mix_update_needed = false;
	}
}

void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);

	 // ADDED
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	float tilt = float(_vtol_vehicle_status.tiltrotor_tilt);
	const float Tilt[4] = {tilt, tilt, tilt, tilt};
	_actuator_known_sp = matrix::Vector<float, 4>(Tilt);
	_control_known_sp = _effectiveness_known * _actuator_known_sp;
	_actuator_unknown_sp = _actuator_trim_unknown + _mix_unknown * ( _control_sp -  _control_known_sp - _control_trim_unknown);
	
	const float act_sp[NUM_ACTUATORS] = {_actuator_unknown_sp(0), _actuator_unknown_sp(1), _actuator_unknown_sp(2), _actuator_unknown_sp(3), _actuator_known_sp(0), _actuator_known_sp(1), _actuator_known_sp(2), _actuator_known_sp(3), _actuator_unknown_sp(4), _actuator_unknown_sp(5), _actuator_unknown_sp(6), _actuator_unknown_sp(7), 0.0f, 0.0f, 0.0f, 0.0f};
	
	_actuator_sp_uk = matrix::Vector<float, NUM_ACTUATORS>(act_sp);
	// ADDED
	
	//printf("actuator_trim = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", double(_actuator_trim(0)),double(_actuator_trim(1)),double(_actuator_trim(2)),double(_actuator_trim(3)),double(_actuator_trim(4)),double(_actuator_trim(5)),double(_actuator_trim(6)), double(_actuator_trim(7)));
	//printf("control_sp = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f\n", double(_control_sp(0)),double(_control_sp(1)),double(_control_sp(2)),double(_control_sp(3)),double(_control_sp(4)),double(_control_sp(5)));
	// printf("_control_known_sp = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f\n", double(_control_known_sp(0)),double(_control_known_sp(1)),double(_control_known_sp(2)),double(_control_known_sp(3)),double(_control_known_sp(4)),double(_control_known_sp(5)));
	// printf("_control_trim_unknown = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f\n", double(_control_trim_unknown(0)),double(_control_trim_unknown(1)),double(_control_trim_unknown(2)),double(_control_trim_unknown(3)),double(_control_trim_unknown(4)),double(_control_trim_unknown(5)));
	// printf("_control_trim = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f\n", double(_control_trim(0)),double(_control_trim(1)),double(_control_trim(2)),double(_control_trim(3)),double(_control_trim(4)),double(_control_trim(5)));
	// printf("_actuator_known_sp = %.3f, %.3f, %.3f,%.3f\n", double(_actuator_known_sp(0)),double(_actuator_known_sp(1)),double(_actuator_known_sp(2)),double(_actuator_known_sp(3)));
	// printf("_actuator_sp = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", double(_actuator_sp(0)),double(_actuator_sp(1)),double(_actuator_sp(2)),double(_actuator_sp(3)),double(_actuator_sp(4)),double(_actuator_sp(5)),double(_actuator_sp(6)), double(_actuator_sp(7)));
	// printf("_actuator_sp_uk = %.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n\n\n\n", double(_actuator_sp_uk(0)),double(_actuator_sp_uk(1)),double(_actuator_sp_uk(2)),double(_actuator_sp_uk(3)),double(_actuator_sp_uk(4)),double(_actuator_sp_uk(5)),double(_actuator_sp_uk(6)), double(_actuator_sp_uk(7)));
	// Clip
	clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated = _effectiveness * _actuator_sp;
}
