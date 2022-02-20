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

	if (failed) {

		printf("failed actuator: %d\n", _actuator_failure_id);
		known_ind.push_back(_actuator_failure_id-1);
		std::sort(known_ind.begin(), known_ind.end());
		_actuator_trim(_actuator_failure_id-1) = _actuator_failure_val;
		failed = false;
		printf("known_id = [");
		for(int i = 0; i < int(known_ind.size()); ++i) {
			printf("%d, ", known_ind[i]);
		}
		printf("]\n");
	}

	_effectiveness = effectiveness;
	int sz;
	getNullSpace(_effectiveness, _nullspace, sz);

	_actuator_trim = actuator_trim;
	clipActuatorSetpoint(_actuator_trim);
	_control_trim = _effectiveness * _actuator_trim;


	float act_trim_k [NUM_ACTUATORS];
	int ind = 0;
	for(int i = 0; i < NUM_ACTUATORS; ++i) {
		if(_actuator_failure_id) {
			if (i == known_ind[ind]) {
				act_trim_k[i] = _actuator_trim(i);
				ind++;
			}
			else {
				act_trim_k[i] = 0.0f;
			}
		}
		else
		{
			act_trim_k[i] = 0.0f;
		}

	}
	_actuator_trim_known = matrix::Vector<float, NUM_ACTUATORS>(act_trim_k);
	_actuator_trim_unknown = _actuator_trim - _actuator_trim_known;
	float eff_known[NUM_AXES][NUM_ACTUATORS];
	for (int j = 0; j < NUM_AXES; ++j) {
		ind = 0;
		for(int i = 0; i < NUM_ACTUATORS; ++i) {
			if (_actuator_failure_id)
			{
				if(i == known_ind[ind]) {
					eff_known[j][i] = _effectiveness(j, i);
					ind++;
				}
				else {
					eff_known[j][i] = 0.0f;
				}
			}
			else
			{
				eff_known[j][i] = 0.0f;
			}


		}
	}
	_effectiveness_known = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> (eff_known);
	_effectiveness_unknown = _effectiveness - _effectiveness_known;

	_control_trim_known = _effectiveness_known * _actuator_trim_known;
	_control_trim_unknown = _effectiveness_unknown * _actuator_trim_unknown;

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
	float mx = -100000.f;
	float mn = 100000.f;
	float eps = 0.01;
	for (int i = 0; i < _num_actuators; i++) {
		if(i < 4)
		{
			if (actuator(i) < _actuator_min(i) && actuator(i) < mn) {
				mn = actuator(i);

			} else if (actuator(i) > _actuator_max(i) && actuator(i) > mx) {
				mx = actuator(i);
			}
		}
		else
		{
			if (_actuator_max(i) < _actuator_min(i)) {
			actuator(i) = _actuator_trim(i);

			} else if (actuator(i) < _actuator_min(i)) {
				actuator(i) = _actuator_min(i);

			} else if (actuator(i) > _actuator_max(i)) {
				actuator(i) = _actuator_max(i);
			}
		}
	}
	if (fabs(mx + 100000.f) > eps)
	{
		for (size_t i = 0; i < 4; i++)
		{
			actuator(i) = actuator(i)*_actuator_max(i)/mx;
		}

	}
	else if (fabs(mn - 100000) > eps)
	{
		for (size_t i = 0; i < 4; i++)
		{
			actuator(i) = actuator(i)*_actuator_min(i)/mn;
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

void
ControlAllocation::getNullSpace(const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &m, matrix::Matrix<float, ControlAllocation::NUM_ACTUATORS - 4, ControlAllocation::NUM_ACTUATORS - 4> &nullspace, int &nullsize)
{
	alglib::real_2d_array a = matrixToAlglib(m);

	alglib::ae_int_t vtneeded = 2; // Determines if vt matrix is needed
	alglib::ae_int_t uneeded = 1; // Determines if u matrix is needed
	alglib::ae_int_t additionalmemory = 2; //If the parameter:
                     			       // * equals 0, the algorithm doesn't use additional
                       			       // memory (lower requirements, lower performance).
		            		       // * equals 1, the algorithm uses additional
		            		       // memory of size min(M,N)*min(M,N) of real numbers.
		            		       // It often speeds up the algorithm.
		            		       // * equals 2, the algorithm uses additional
		            		       // memory of size M*min(M,N) of real numbers.
		            		       // It allows to get a maximum performance.
		            		       // The recommended value of the parameter is 2.
	alglib::real_1d_array w;
	alglib::real_2d_array vt;
	alglib::real_2d_array u;
	// rmatrixsvd find A = U W V^T

	bool success = alglib::rmatrixsvd(a, a.rows(), a.cols(), uneeded, vtneeded, additionalmemory, w, u, vt);
	int non_zero_eigens = 0;

	if(success) {
		for(int i = 0; i < ControlAllocation::NUM_AXES; ++i) {
			if(w(i) > 0.001)
			{
				non_zero_eigens++;
			}
		}
	}
	nullsize = ControlAllocation::NUM_ACTUATORS - 4 - non_zero_eigens;

	for(int i = 0; i < ControlAllocation::NUM_ACTUATORS - 4; ++i) {
		for(int j = 0; j < nullsize; ++j) {
			nullspace(i, j) = vt(j + non_zero_eigens, i);
		}
	}
	// printf("W = [ %f  %f  %f  %f  %f  %f ]\n", w(0), w(1), w(2), w(3), w(4), w(5));
}


alglib::real_2d_array
ControlAllocation::matrixToAlglib(const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &m)
{
	alglib::real_2d_array c;
	int s=6, t=12, i , j;
	c.setlength(s, t);
	for ( i = 0; i < s; i++)
	{
		for ( j = 0; j < t; j++)
		{
			c[i][j] = m(i, j);
		}
	}
	return c;
}
matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::alglibToMatrix(const alglib::real_2d_array &m)
{
	matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> c;
	for(int i = 0; i < ControlAllocation::NUM_AXES; ++i) {
		for(int j = 0; j < ControlAllocation::NUM_ACTUATORS; ++j) {
			if(j >= ControlAllocation::NUM_ACTUATORS + 4) c(i, j) = 0;
			else c(i, j) = m (i, j);
		}
	}
	return c;
}

void ControlAllocation::printAlglib(const alglib::real_2d_array &a)
{
	int i, j;
	for ( i = 0; i < a.rows(); i++)
	{
		for ( j = 0; j < a.cols(); j++)
		{
			printf("\t %.6f, ", double(a(i, j)));
		}
		printf("\n");
	}
}
