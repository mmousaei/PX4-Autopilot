/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessStandardVTOL.hpp
 *
 * Actuator effectiveness for standard VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessStandardVTOL.hpp"

ActuatorEffectivenessStandardVTOL::ActuatorEffectivenessStandardVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

bool
ActuatorEffectivenessStandardVTOL::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
	if (!_updated) {
		return false;
	}
	float Tz = -6.5f;
	float Tx = 6.5f; //6.5f;
	float tau_x = 1.5925f; //1.5925f; //2.3198f;
	float tau_x2 = 1.5925f; //1.21875f; //2.3198f;
	float tau_y = 0.98475; //0.98475; //2.3198f;
	float tau_z = 0.325f;
	float ale = 24.1014*0.1;
	float ele = 114.2482*0.1*0.5*0.5*0.5;
	float Tx_tran = 6.5*0.5;
	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{-tau_x,  tau_x2,  tau_x, -tau_x2, 0.f, 0.0f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ tau_y, -tau_y,  tau_y, -tau_y, 0.f, 0.f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ tau_z,  tau_z, -tau_z, -tau_z, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, Tx*0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{Tz, Tz, Tz, Tz, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{ 0.f, 0.f, 0.f, 0.f, 0.f, -ale, ale, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, ele, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, Tx_tran, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{ 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}

	case FlightPhase::TRANSITION_HF_TO_FF: {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{ -tau_x,  tau_x,  tau_x, -tau_x, 0.f, -ale, ale, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  tau_y, -tau_y,  tau_y, -tau_y, 0.f, 0.f, 0.f, ele, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  tau_z,  tau_z, -tau_z, -tau_z, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f,  0.f, 0.f, Tx_tran, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{Tz, Tz, Tz, Tz, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF: {
			const float standard_vtol[NUM_AXES][NUM_ACTUATORS] = {
				{ -tau_x,  tau_x,  tau_x, -tau_x, 0.f, -ale, ale, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  tau_y, -tau_y,  tau_y, -tau_y, 0.f, 0.f, 0.f, ele, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  tau_z,  tau_z, -tau_z, -tau_z, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f,  0.f, 0.f, Tx_tran, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
				{Tz, Tz, Tz, Tz, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
			};
			matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(standard_vtol);
			break;
		}
	}

	_updated = false;
	return true;
}

void
ActuatorEffectivenessStandardVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);
	_updated = true;

}
