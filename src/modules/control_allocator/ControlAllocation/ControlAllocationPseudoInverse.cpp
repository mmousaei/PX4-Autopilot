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
	// _optimize_sample();

	// Allocate
	// _actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);

	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);

	// Clip
	// clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated =  _effectiveness * _actuator_sp;
	matrix::Vector<float, NUM_AXES> _control_unallocated;
	_control_unallocated = (_control_sp - _control_allocated);



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
		// printf("unallocated control:\n");
		// _control_unallocated.T().print();

		matrix::Vector<float, NUM_ACTUATORS - 4> _actuator_opt;
		float act_opt[NUM_ACTUATORS - 4];
		for (int i = 0; i < NUM_ACTUATORS - 4; i++)
		{
			act_opt[i] = _actuator_sp(i);
		}
		_actuator_opt = matrix::Vector<float, NUM_ACTUATORS - 4> (act_opt);
		matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint;
		linear_constraint = _actuator_opt.T() * _nullspace * 2;
		_optimize_allocation(linear_constraint, _actuator_opt);
		// printf("optimizing here!\n");
		bool isNan_opt = false;
		for (size_t i = 0; i < _null_size; i++)
		{
			if(isnan(_lambda_sol(i))) isNan_opt = true;
		}
		if(!isNan_opt)
		{
			_last_lambda_sol = _lambda_sol;
			_last_lambda_init = true;
		}
		matrix::Vector<float, NUM_ACTUATORS - 4> _lambda;
		int i;
		for (i = 0; i < NUM_ACTUATORS - 4; i++)
		{
			if(i < _null_size)
			{
				_lambda(i) = _lambda_sol(i);
			}
			else
			{
				_lambda(i) = 0.0f;
			}

		}
		matrix::Vector<float, NUM_ACTUATORS> _actuator_optimized;
		for (i = 0; i < NUM_ACTUATORS; i++)
		{
			if(i < NUM_ACTUATORS - 4)
			{
				if(i < (_actuator_failure_id-1))
				{
					_actuator_optimized(i) = matrix::Vector<float, NUM_ACTUATORS - 5>(_nullspace_failed * _lambda)(i);
				}
				else if (i == (_actuator_failure_id-1))
				{
					_actuator_optimized(i) = 0.f;
				}
				else
				{
					_actuator_optimized(i) = matrix::Vector<float, NUM_ACTUATORS - 5>(_nullspace_failed * _lambda)(i-1);
				}


			}
		}

		if(!isNan_opt)
		{
			_actuator_sp +=_actuator_optimized;
		}

	}
	clipActuatorSetpoint(_actuator_sp);
	_control_allocated =  _effectiveness * _actuator_sp;
	_control_unallocated = (_control_sp - _control_allocated);
	printf("unallocated control:\n");
	_control_unallocated.T().print();
	printf("_act_sp\n");
	_actuator_sp.T().print();

	// printf("max:\n");
	// _actuator_max.T().print();
	// printf("min:\n");
	// _actuator_min.T().print();
	// // Save into csv TODO: add a QGC bool param to control save or not save
	// _vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	// _airspeed_validated_sub.update(&_airspeed_validated);
	// cFile.open("myFile.csv", std::ios_base::app);
	// cFile << std::endl << double(_control_sp(0)) << ", "<< double(_control_sp(1)) << ", "<< double(_control_sp(2)) << ", "<< double(_control_sp(3)) << ", "<< double(_control_sp(4)) << ", "<< double(_control_sp(5))<< ", " << double(_actuator_sp(0))<< ", "<< double(_actuator_sp(1))<< ", "<< double(_actuator_sp(2))<< ", "<< double(_actuator_sp(3))<< ", "<< double(_actuator_sp(4))<< ", "<< double(_actuator_sp(5))<< ", "<< double(_actuator_sp(6))<< ", "<< double(_actuator_sp(7))<< ", "<< double(_actuator_sp(8))<< ", "<< double(_actuator_sp(9))<< ", "<< double(_actuator_sp(10))<< ", "<< double(_actuator_sp(11))<< ", " <<double(_airspeed_validated.calibrated_airspeed_m_s)<<", "<<double(_vtol_vehicle_status.roll)<<", "<<double(_vtol_vehicle_status.pitch)<< std::endl;
	// cFile.close();

}

void ControlAllocationPseudoInverse::_optimize_sample()
{
// This example demonstrates minimization of nonconvex function
//     F(x0,x1) = -(x0^2+x1^2)
// subject to constraints x0,x1 in [1.0,2.0]
// Exact solution is [x0,x1] = [2,2].
//
// Non-convex problems are harded to solve than convex ones, and they
// may have more than one local minimum. However, ALGLIB solves may deal
// with such problems (altough they do not guarantee convergence to
// global minimum).
//
// IMPORTANT: this solver minimizes  following  function:
//     f(x) = 0.5*x'*A*x + b'*x.
// Note that quadratic term has 0.5 before it. So if you want to minimize
// quadratic function, you should rewrite it in such way that quadratic term
// is multiplied by 0.5 too.
//
// For example, our function is f(x)=-(x0^2+x1^2), but we rewrite it as
//     f(x) = 0.5*(-2*x0^2-2*x1^2)
// and pass diag(-2,-2) as quadratic term - NOT diag(-1,-1)!
//
	alglib::real_2d_array a;
	double data_2_2[4] = {2.0, 0.0, 0.0, 2.0};
	a.setcontent(2, 2, data_2_2);
	alglib::real_1d_array b;
	double data_2[2] = {-6.0, -4.0};
	b.setcontent(2, data_2);
	alglib::real_1d_array x0;
	data_2[0] = 0.0;
	data_2[1] = 1.0;
	x0.setcontent(2, data_2);
	alglib::real_1d_array s;
	data_2[0] = 1.0;
	s.setcontent(2, data_2);
	alglib::real_1d_array bndl;
	data_2[0] = 0.0;
	data_2[1] = 0.0;
	bndl.setcontent(2, data_2);
	alglib::real_1d_array bndu;
	data_2[0] = 2.5;
	data_2[1] = 2.5;
	bndu.setcontent(2, data_2);


	alglib::real_1d_array x;
	alglib::minqpstate state;
	alglib_impl::ae_state _state;
	alglib_impl::ae_state_init(&_state);
	alglib::minqpreport rep;


	alglib::minqpcreate(2, state);
	alglib_impl::minqpsetquadraticterm(const_cast<alglib_impl::minqpstate*>(state.c_ptr()), const_cast<alglib_impl::ae_matrix*>(a.c_ptr()), isupper, &_state);
	alglib::minqpsetlinearterm(state, b);
	alglib::minqpsetstartingpoint(state, x0);
	alglib::minqpsetbc(state, bndl, bndu);

	alglib::minqpsetscale(state, s);

	// Uncomment to use BLEIC optimization method
	// minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
	alglib::minqpsetalgoquickqp(state, 0.0, 0.0, 0.0, 0, true);
	alglib::minqpoptimize(state);
	alglib::minqpresults(state, x, rep);
	printf("solution = [%f, %f]\n", x[0], x[1]); // EXPECTED: [2.5,2]
}

void ControlAllocationPseudoInverse::_optimize_allocation(matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint, matrix::Vector<float, NUM_ACTUATORS - 4> actuator_opt)
{

	alglib::real_2d_array a;
	a.setlength(_null_size, _null_size);
	double zeros[_null_size*_null_size] = {0};
	a.setcontent(_null_size, _null_size, zeros);
	for (int i = 0; i < _null_size; i++)
	{
		a(i, i) = 2.0;
	}
	alglib::real_1d_array b = matrixToAlglib(linear_constraint);
	alglib::real_1d_array x0;
	double x0_arr[_null_size];
	for (int i = 0; i < _null_size; i++)
	{
		if(!_last_lambda_init)
		{
			x0_arr[i] = 1;
		}
		else
		{
			x0_arr[i] = _last_lambda_sol(i);
		}

		// x0_arr[i] = actuator_opt(i); // TODO: find the best starting values

	}
	x0.setcontent(_null_size, x0_arr);
	alglib::real_1d_array s;
	s.setlength(_null_size);
	for (size_t i = 0; i < _null_size; i++)
	{
		s(i) = 12.8;
	}

	alglib::real_2d_array C;
	alglib::integer_1d_array ct;
	int constraint_size;
	if(_actuator_failure_id) constraint_size = 22; else constraint_size = 24;
	C.setlength(constraint_size, _null_size + 1);
	ct.setlength(constraint_size);
	int i, j;
	for (i = 0; i < constraint_size; i++)
	{
		for (j = 0; j < _null_size + 1; j++)
		{
			// RHS of constraint
			if(j == _null_size)
			{
				if(i < (constraint_size/2))
				{
					if(i < (_actuator_failure_id-1))
					{
						C(i, j) = _actuator_max(i) - actuator_opt(i);
					}
					else
					{
						C(i, j) = _actuator_max(i+1) - actuator_opt(i+1);
					}
					ct(i) = -1;
				}
				else
				{
					if((i%(constraint_size/2)) < (_actuator_failure_id-1))
					{
						C(i, j) = _actuator_min(i%(constraint_size/2)) - actuator_opt(i%(constraint_size/2));
					}
					else
					{
						C(i, j) = _actuator_min((i%(constraint_size/2))+1) - actuator_opt((i%(constraint_size/2))+1);
					}
					ct(i) = 1;
				}

			}
			// LHS of constraint
			else
			{
				if(_actuator_failure_id)
				{
					C(i, j) = _nullspace_failed(i%(constraint_size/2), j);
				}
				else
				{
					C(i, j) = _nullspace(i%(constraint_size/2), j);
				}

			}

		}

	}
	// printf("null:\n");
	// _nullspace_failed.print();
	// printf("constraints:\n");
	// printAlglib(C);
	// printf("linear:\n");
	// printAlglib(b);
	// printf("x0:\n");
	// printAlglib(x0);
	alglib::real_1d_array x;
	alglib::minqpstate state;
	alglib_impl::ae_state _state;
	alglib_impl::ae_state_init(&_state);
	alglib_impl::ae_state _lc_state;
	alglib_impl::ae_state_init(&_lc_state);
	alglib::minqpreport rep;

	alglib::minqpcreate(_null_size, state);
	alglib_impl::minqpsetquadraticterm(const_cast<alglib_impl::minqpstate*>(state.c_ptr()), const_cast<alglib_impl::ae_matrix*>(a.c_ptr()), isupper, &_state);
	alglib::minqpsetstartingpoint(state, x0);

	// alglib::minqpsetlc(state, C, ct);
	alglib_impl::minqpsetlc(const_cast<alglib_impl::minqpstate*>(state.c_ptr()), const_cast<alglib_impl::ae_matrix*>(C.c_ptr()), const_cast<alglib_impl::ae_vector*>(ct.c_ptr()), C.rows(), &_lc_state);

	// Set scale: currectly set to 1/sqrt(diag(a)) TODO: uncomment the commented one and find the best scaling factors
	// alglib::minqpsetscale(state, s);
	alglib::minqpsetscaleautodiag(state);

	alglib::minqpsetalgobleic(state, 0.2, 0.2, 0.2, 0);
	// alglib::minqpsetalgoquickqp(state, 0.001, 0.001, 0.001, 0, true);
	alglib::minqpoptimize(state);
	alglib::minqpresults(state, x, rep);

	if(rep.inneriterationscount > mx) mx = rep.inneriterationscount;
	printf("solution = \n");
	printAlglib(x);
	printf("opt report: \ninner cnt: %d\nouter cnt:%d\ntermination type:%d\n", rep.inneriterationscount, rep.outeriterationscount, rep.terminationtype);
	printf("max iter = %d\n", mx);
	_lambda_sol = x;

}
