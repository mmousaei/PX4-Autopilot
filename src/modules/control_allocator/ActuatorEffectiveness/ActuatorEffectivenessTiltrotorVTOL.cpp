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
 * @file ActuatorEffectivenessTiltrotorVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltrotorVTOL.hpp"

ActuatorEffectivenessTiltrotorVTOL::ActuatorEffectivenessTiltrotorVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}
bool
ActuatorEffectivenessTiltrotorVTOL::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
	if (!_updated) {
		return false;
	}

	// Trim
	float tilt = 0.0f;
	float airspeed = 0.0f;
	// PX4_ERR("NUM_AXES: "+string(NUM_AXES));

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			tilt = 0.0f;
			airspeed = 0.0f;
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			tilt = 0.65;
			airspeed = 20.0f;
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF: {
			tilt = 0.6;//0.5759586531581288f;
			airspeed = 20.0f;
			break;
		}
	}

	// Trim: half throttle, tilted motors
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	_airspeed_validated_sub.update(&_airspeed_validated);

	airspeed = float(_airspeed_validated.calibrated_airspeed_m_s);
	//if (_flight_phase == FlightPhase::FORWARD_FLIGHT){airspeed = 20.0f;}
	tilt = float(_vtol_vehicle_status.tiltrotor_tilt);
	_trim(0) = 0.5f;
	_trim(1) = 0.5f;
	_trim(2) = 0.5f;
	_trim(3) = 0.5f;
	_trim(4) = tilt;//*1.570796f;
	_trim(5) = tilt;//*1.570796f;
	_trim(6) = tilt;//*1.570796f;
	_trim(7) = tilt;//*1.570796f;
	printf("alan injaaaaaaaaaaaaaaam  =  %f,  %f\n", double(_airspeed_validated.calibrated_airspeed_m_s), double(_vtol_vehicle_status.tiltrotor_tilt));
	float trim4_m = _trim(4)*1.570796f;
	float trim5_m = _trim(5)*1.570796f;
	float trim6_m = _trim(6)*1.570796f;
	float trim7_m = _trim(7)*1.570796f;

	// // Effectiveness
	// const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
	// 	{-0.5f * cosf(_trim(4)),  0.5f * cosf(_trim(5)),  0.5f * cosf(_trim(6)), -0.5f * cosf(_trim(7)), 0.5f * _trim(0) *sinf(_trim(4)), -0.5f * _trim(1) *sinf(_trim(5)), -0.5f * _trim(2) *sinf(_trim(6)), 0.5f * _trim(3) *sinf(_trim(7)), -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.5f * cosf(_trim(4)), -0.5f * cosf(_trim(5)),  0.5f * cosf(_trim(6)), -0.5f * cosf(_trim(7)), -0.5f * _trim(0) *sinf(_trim(4)),  0.5f * _trim(1) *sinf(_trim(5)), -0.5f * _trim(2) *sinf(_trim(6)), 0.5f * _trim(3) *sinf(_trim(7)), 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{-0.5f * sinf(_trim(4)),  0.5f * sinf(_trim(5)),  0.5f * sinf(_trim(6)), -0.5f * sinf(_trim(7)), -0.5f * _trim(0) *cosf(_trim(4)), 0.5f * _trim(1) *cosf(_trim(5)), 0.5f * _trim(2) *cosf(_trim(6)), -0.5f * _trim(3) *cosf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.25f * sinf(_trim(4)), 0.25f * sinf(_trim(5)), 0.25f * sinf(_trim(6)), 0.25f * sinf(_trim(7)), 0.25f * _trim(0) *cosf(_trim(4)), 0.25f * _trim(1) *cosf(_trim(5)), 0.25f * _trim(2) *cosf(_trim(6)), 0.25f * _trim(3) *cosf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
	// 	{-0.25f * cosf(_trim(4)), -0.25f * cosf(_trim(5)), -0.25f * cosf(_trim(6)), -0.25f * cosf(_trim(7)), 0.25f * _trim(0) *sinf(_trim(4)), 0.25f * _trim(1) *sinf(_trim(5)), 0.25f * _trim(2) *sinf(_trim(6)), 0.25f * _trim(3) *sinf(_trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	// };
	// matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);

	// // Temporarily disable a few controls (WIP)
	// for (size_t j = 4; j < 8; j++) {
	// 	matrix(3, j) = 0.0f;
	// 	matrix(4, j) = 0.0f;
	// 	matrix(5, j) = 0.0f;
	// }



	float Px_0 = 0.1515f;
	float Py_0 = 0.245f;
	float Pz_0 = 0.0f;
	float Px_1 = -0.1515f;
	float Py_1 = -0.245f;
	float Pz_1 = 0.0f;
	float Px_2 = 0.1515f;
	float Py_2 = -0.245f;
	float Pz_2 = 0.0f;
	float Px_3 = -0.1515f;
	float Py_3 = 0.245f;
	float Pz_3 = 0.0f;
	float Ct = 6.5f;
	float Km = 0.05f;
	float ro = 1.225f;
	float q_bar = ro * airspeed * airspeed / 2;
	float S = 0.4266f;
	float b = 2.0f;
	float c_bar = 0.2f;
	float Cla = 0.11730*0.1;
	float Cme = 0.55604*0.1*0.5;//*0.5*0.5;
	float Cnr = 0.0;//0.08810f;

	// 			w_0							  w_1							w_2								w_3								theta_0									  theta_1										theta_2												theta_3								   delta_a left		   delta_a right	   delta_e			   delta_r
	const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
		{-Py_0 * Ct*cosf(trim4_m) - Ct * Km * sinf(trim4_m),	-Py_1 * Ct*cosf(trim5_m) - Ct * Km * sinf(trim5_m),	 -Py_2 * Ct*cosf(trim6_m) + Ct * Km * sinf(trim6_m),	-Py_3 * Ct*cosf(trim7_m) + Ct * Km * sinf(trim7_m),	 Py_0 * Ct*_trim(0) *sinf(trim4_m) - Ct * Km * _trim(0) *cosf(trim4_m),    	     Py_1 * Ct*_trim(1) *sinf(trim5_m) - Ct * Km * _trim(1) *cosf(trim5_m),		 		Py_2 * Ct*_trim(2) *sinf(trim6_m) + Ct * Km * _trim(2) *cosf(trim6_m),		         	Py_3 * Ct*_trim(3) *sinf(trim7_m) + Ct * Km * _trim(3) *cosf(trim7_m),  			-q_bar*S*b*Cla,		 q_bar*S*b*Cla,		 0.f, 		 		 0.f, 			0.f, 0.f, 0.f, 0.f},
		{ Ct*(Px_0 * cosf(trim4_m) + Pz_0 * sinf(trim4_m)),    Ct*(Px_1 * cosf(trim5_m) + Pz_1 * sinf(trim5_m)),	  Ct*(Px_2 * cosf(trim6_m) + Pz_2 * sinf(trim6_m)),	 Ct*(Px_3 * cosf(trim7_m) + Pz_3 * sinf(trim7_m)),	 Ct*_trim(0) *(-Px_0 * sinf(trim4_m) + Pz_0 * cosf(trim4_m)), 	     		     Ct*_trim(1) *(-Px_1 *sinf(trim5_m) + Pz_1 * cosf(trim5_m)),					Ct*_trim(2) *(-Px_2 *sinf(trim6_m) + Pz_2 * cosf(trim6_m)),					Ct*_trim(3) *(-Px_3 *sinf(trim7_m) + Pz_3 *cosf(trim7_m)),  					 0.f, 		 	 0.f, 		 	 q_bar*S*c_bar*Cme,		 0.f, 			0.f, 0.f, 0.f, 0.f},
		{-Py_0 * Ct*sinf(trim4_m) + Ct * Km * cosf(trim4_m),	-Py_1 * Ct*sinf(trim5_m) + Ct * Km * cosf(trim5_m),	 -Py_2 * Ct*sinf(trim6_m) - Ct * Km * cosf(trim6_m),	-Py_3 * Ct*sinf(trim7_m) - Ct * Km * cosf(trim7_m),	-Py_0 * Ct*_trim(0) *cosf(trim4_m) - Ct * Km * _trim(0) *sinf(trim4_m),   	     -Py_1 * Ct*_trim(1) *cosf(trim5_m) - Ct * Km * _trim(1) *sinf(trim5_m),			-Py_2 * Ct*_trim(2) *cosf(trim6_m) + Ct * Km * _trim(2) *sinf(trim6_m), 		-Py_3 * Ct*_trim(3) *cosf(trim7_m) + Ct * Km * _trim(3) *sinf(trim7_m),  			 0.f, 		 	 0.f, 		 	 0.f, 		 		 q_bar*S*b*Cnr, 	0.f, 0.f, 0.f, 0.f},
		{ Ct * sinf(trim4_m),	 				 Ct * sinf(trim5_m),					 Ct * sinf(trim6_m),					 Ct * sinf(trim7_m),					 Ct * _trim(0) *cosf(trim4_m), 					     		     Ct * _trim(1) *cosf(trim5_m),							 		Ct * _trim(2) *cosf(trim6_m), 								 	Ct * _trim(3) *cosf(trim7_m),   								 0.f, 		 	 0.f, 		 	 0.f, 		 		 0.f, 			0.f, 0.f, 0.f, 0.f},
		{ 0.f,  			 			 0.f,  							 0.f,  							 0.f,	 						 0.f,		 		  					     		     0.f, 			      							 		0.f, 			    	  								 0.f, 			           								 0.f, 		 	 0.f, 		 	 0.f, 		 		 0.f, 			0.f, 0.f, 0.f, 0.f},
		{-Ct * cosf(trim4_m),	 			        -Ct * cosf(trim5_m),					-Ct * cosf(trim6_m),					-Ct * cosf(trim7_m),					 Ct * _trim(0) *sinf(trim4_m), 					     		     Ct * _trim(1) *sinf(trim5_m),							 		Ct * _trim(2) *sinf(trim6_m), 								 	Ct * _trim(3) *sinf(trim7_m),   								 0.f, 		 	 0.f, 		 	 0.f, 		 		 0.f, 			0.f, 0.f, 0.f, 0.f}
	};
	matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol);

	_updated = false;
	return true;
}

void
ActuatorEffectivenessTiltrotorVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;
}
