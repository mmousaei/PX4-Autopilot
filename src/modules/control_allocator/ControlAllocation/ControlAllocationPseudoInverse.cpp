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
	// _actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);

	 // ADDED
	// _vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	// float act_known[NUM_ACTUATORS];
	// int cnt = 0;
	// for(int i = 0; i < NUM_ACTUATORS; ++i) {
	// 	if (i == known_ind[cnt]) {
	// 		act_known[i] = _actuator_trim_known(i);
	// 		cnt++;
	// 	}
	// 	else{
	// 		act_known[i] = 0.0f;
	// 	}
	// }
	// _actuator_known_sp = matrix::Vector<float, NUM_ACTUATORS>(act_known);
	// _control_known_sp = _effectiveness_known * _actuator_known_sp;


	// _actuator_unknown_sp = _actuator_trim_unknown + _mix_unknown * ( _control_sp -  _control_known_sp - _control_trim_unknown);

	// const float act_sp[NUM_ACTUATORS] = _actuator_unknown_sp + _actuator_known_sp;

	// _actuator_sp = _actuator_unknown_sp + (_actuator_known_sp);
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
	// printf("_mix:\n");
	// _mix.T().print();

	// if(_vtol_vehicle_status.vtol_in_trans_mode)
	// {
	// 	_actuator_sp = _actuator_unknown_sp + (_actuator_known_sp);
	// }

	// ADDED
	// printf("_actuator_trim + _mix * (_control_sp - _control_trim):\n");
	// (_actuator_trim + _mix * (_control_sp - _control_trim)).T().print();
	// printf("_actuator_known_sp:\n");
	// _actuator_known_sp.T().print();
	// printf("_actuator_unknown_sp:\n");
	// _actuator_unknown_sp.T().print();





	// Clip
	clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated =  _effectiveness * _actuator_sp;
	matrix::Vector<float, NUM_AXES> _control_unallocated;
	_control_unallocated = (_control_sp - _control_allocated);
	// printf("_actuator_sp:\n");
	// _actuator_sp.T().print();
	// printf("_control_sp:\n");
	// _control_sp.T().print();
	// printf("_control_allocated:\n");
	// _control_allocated.T().print();
	// printf("_control_trim:\n");
	// _control_trim.T().print();
	// printf("_effectiveness:\n");
	// _effectiveness.T().print();
	// printf("_effectiveness_unknown:\n");
	// _effectiveness_unknown.T().print();
	if (_actuator_failure_id) {
		float act_known[NUM_ACTUATORS];
		int cnt = 0;
		for(int i = 0; i < NUM_ACTUATORS; ++i) {
			if (i == known_ind[cnt]) {
				act_known[i] = _actuator_trim_known(i);
				cnt++;
			}
			else{
				act_known[i] = 0.0f;
			}
		}
		_actuator_known_sp = matrix::Vector<float, NUM_ACTUATORS>(act_known);
		_control_known_sp = _effectiveness_known * _actuator_known_sp;


		_actuator_unknown_sp = _actuator_trim_unknown + _mix_unknown * ( _control_sp -  _control_known_sp - _control_trim_unknown);
		_actuator_sp = _actuator_unknown_sp + (_actuator_known_sp);

		_control_allocated =  _effectiveness * _actuator_sp;
		_control_unallocated = (_control_sp - _control_allocated);
		// printf("_actuator_sp:\n");
		// _actuator_sp.T().print();
		// printf("_actuator_trim:\n");
		// _actuator_trim.T().print();
		// printf("_control_sp:\n");
		// _control_sp.T().print();
		// printf("_effectiveness:\n");
		// _effectiveness.T().print();
		// printf("_effectiveness_known:\n");
		// _effectiveness_known.T().print();
		// matrix::Vector<float, NUM_AXES> _control_unallocated;
		// _control_unallocated = (_control_sp - _control_allocated);

		// printf("failure: %d, value = %f\n", _actuator_failure_id, double(_actuator_failure_val));
		int cnter = 0;
		int cnt_thresh = 400;
		double Mnorm, Fnorm;
		Mnorm = _control_unallocated(0)*_control_unallocated(0) + _control_unallocated(1)*_control_unallocated(1) + _control_unallocated(2)*_control_unallocated(2);
		// Fnorm = _control_unallocated(3)*_control_unallocated(3) + _control_unallocated(4)*_control_unallocated(4) + _control_unallocated(5)*_control_unallocated(5);
		// printf("Mnorm = %f, Fnorm = %f\n", Mnorm, Fnorm);
		// if (_actuator_failure_id > 8)
		// {
			// while ((_control_unallocated.norm() > float(0.01)) && cnter < cnt_thresh) {
			while (((Mnorm > 0.01) || (Fnorm > 1)) && cnter < cnt_thresh) {
				_actuator_unallocated_sp =  _mix_unknown * (_control_unallocated - _control_known_sp);
				_actuator_sp += _actuator_unallocated_sp;
				clipActuatorSetpoint(_actuator_sp);
				_control_allocated = _effectiveness * _actuator_sp;
				_control_unallocated = (_control_sp - _control_allocated);
				// printf("here re-allocate!: %f\n", double(_control_unallocated.norm()));
				cnter++;
			}
			// printf("\n\n");
		// }
		// printf("norm = %f\n", double(_control_unallocated.norm()));
		// else {
		// 	// while (((Mnorm > 0.2) || (Fnorm > 2)) && cnter < cnt_thresh) {
		// 	while ((_control_unallocated.norm() > float(0.1)) && cnter < cnt_thresh) {
		// 		_actuator_unallocated_sp =  _mix * _control_unallocated;
		// 		_actuator_sp += _actuator_unallocated_sp;
		// 		clipActuatorSetpoint(_actuator_sp); // TODO: make sure this is scale and clip
		// 		_control_allocated = _effectiveness * _actuator_sp;
		// 		_control_unallocated = (_control_sp - _control_allocated);
		// 		// printf("here re-allocate!: %f\n", double(_control_unallocated.norm()));
		// 		cnter++;
		// 	}
		// 	// printf("\n\n");
		// }
		// printf("\nreallocated_actuator_sp:\n");
		// _actuator_sp.T().print();
		// printf("\n\n\n");
	}
	printf("\nreallocated_actuator_sp:\n");
		_actuator_sp.T().print();
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	_airspeed_validated_sub.update(&_airspeed_validated);
	// printf("airspeed = %f, Roll = %f, Pitch = %f\n", double(_airspeed_validated.calibrated_airspeed_m_s), double(_vtol_vehicle_status.roll), double(_vtol_vehicle_status.pitch));
	// printf("c(3) =\t%.2f, c(4) =\t%.2f, c(5) =\t%.2f\n", double(_control_sp(3)), double(_control_sp(4)), double(_control_sp(5)) );
	cFile.open("myFile.csv", std::ios_base::app);
	cFile << std::endl << double(_control_sp(0)) << ", "<< double(_control_sp(1)) << ", "<< double(_control_sp(2)) << ", "<< double(_control_sp(3)) << ", "<< double(_control_sp(4)) << ", "<< double(_control_sp(5))<< ", " << double(_actuator_sp(0))<< ", "<< double(_actuator_sp(1))<< ", "<< double(_actuator_sp(2))<< ", "<< double(_actuator_sp(3))<< ", "<< double(_actuator_sp(4))<< ", "<< double(_actuator_sp(5))<< ", "<< double(_actuator_sp(6))<< ", "<< double(_actuator_sp(7))<< ", "<< double(_actuator_sp(8))<< ", "<< double(_actuator_sp(9))<< ", "<< double(_actuator_sp(10))<< ", "<< double(_actuator_sp(11))<< ", " <<double(_airspeed_validated.calibrated_airspeed_m_s)<<", "<<double(_vtol_vehicle_status.roll)<<", "<<double(_vtol_vehicle_status.pitch)<< std::endl;
	cFile.close();
	// printf("un allocated = %f\n", double(_control_unallocated.norm()));

}
