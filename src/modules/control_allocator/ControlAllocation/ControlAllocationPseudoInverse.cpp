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
	_vtol_vehicle_status_sub.update(&_vtol_vehicle_status);
	_airspeed_validated_sub.update(&_airspeed_validated);
	float tilt_degree = _vtol_vehicle_status.tiltrotor_tilt / 2.0f * 180.0f;
	float airspeed = _airspeed_validated.calibrated_airspeed_m_s;
	float ro = 1.225f;
	float q_bar = ro * airspeed * airspeed / 2;
	float S = 0.4266f;
	float alpha = 1 * 3.1415 / 180;
	float Cl = 0.11 + 0.11 * alpha;
	float Cd = 0.01 + 0.2 * alpha * alpha;
	// float b = 2.0f;
	// float Cme = 0.55604*0.1*0.5*0.5*0.5;
	_aero_wrench(5) = q_bar * S * Cl;
	_aero_wrench(3) = q_bar * S * Cd;
	// _aero_wrench(1) = q_bar * S * b * Cme * 0.2;
	// printf("lift:\n");
	// _aero_wrench.T().print();
	// printf("c sp:\n");
	// _control_sp.T().print();
	_control_sp += _aero_wrench;

	// Allocate
	// _actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);

	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
	// clipActuatorSetpoint(_actuator_sp);
	_actuator_sp_no_opt = _actuator_sp;
	_actoator_sp_original = _actuator_sp;

	// // Clip
	// clipActuatorSetpoint(_actuator_sp);

	// Compute achieved control
	_control_allocated =  _effectiveness * _actuator_sp;
	matrix::Vector<float, NUM_AXES> _control_unallocated;
	_control_unallocated = (_control_sp - _control_allocated);


	_control_allocated_without_opt = _control_allocated;



	// matrix::Matrix<float, 12, 1> act;
	// for (size_t i = 0; i < 12; i++)
	// {
	// 	act(i, 0) = _actuator_sp(i);
	// }
	// matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint;
	// linear_constraint = act.T() * _nullspace * 2;
	// int termination_type = _optimize_allocation_simple_cost(linear_constraint);

///////////////////////////////////////// FAILURE ////////////////////////////////////////////////
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
		// clipActuatorSetpoint(_actuator_sp);
		_actuator_sp_no_opt = _actuator_sp;
		_actuator_sp = _actuator_unknown_sp + (_actuator_known_sp);
		_control_allocated =  _effectiveness * _actuator_sp;
		_control_unallocated = (_control_sp - _control_allocated);
		// printf("unallocated control:\n");
		// _control_unallocated.T().print();

		matrix::Vector<float, NUM_ACTUATORS - 5> _actuator_opt;
		float act_opt[NUM_ACTUATORS - 5];
		float act_err[NUM_ACTUATORS - 4];
		for (int i = 0; i < NUM_ACTUATORS - 5; i++)
		{
			act_err[i] = _actuator_sp(i);
			if(i < _actuator_failure_id)
			{
				act_opt[i] = _actuator_sp(i);
				if(i < 4)
				{
					act_opt[i] = _actuator_sp(i) - 0.3;
				}
				else if(i < 8 &&  i >= 4)
				{
					act_opt[i] = _actuator_sp(i) - 1.57;
				}

			}
			else
			{
				act_opt[i] = _actuator_sp(i+1);
				if(i < 4)
				{
					act_opt[i] = _actuator_sp(i+1) - 0.3;
				}
				else if(i < 8 &&  i >= 4)
				{
					act_opt[i] = _actuator_sp(i) - 1.57;
				}
			}

		}
		_actuator_opt = matrix::Vector<float, NUM_ACTUATORS - 5> (act_opt);
		matrix::Vector<float, NUM_ACTUATORS - 4> _actuator_err = matrix::Vector<float, NUM_ACTUATORS - 4> (act_err);
		_actuator_err -= _actuator_last;
		matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint;
		linear_constraint = _actuator_opt.T() * _nullspace_failed * 2;
		// int termination_type = _optimize_allocation_simple_cost(linear_constraint);
		int termination_type = _optimize_allocation_err_cost(_actuator_err);
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
		if(!isNan_opt && termination_type > 0 && termination_type < 5)
		{
			printf("_actuator_optimized:\n");
			_actuator_optimized.T().print();
			_actuator_sp +=_actuator_optimized;
		}

	}













	// matrix::Vector<float, NUM_ACTUATORS - 4> _lambda;
	// int i;
	// for (i = 0; i < NUM_ACTUATORS - 4; i++)
	// {
	// 	_lambda(i) = _lambda_sol(i);
	// }
	// matrix::Vector<float, NUM_ACTUATORS> _actuator_optimized;
	// for (i = 0; i < NUM_ACTUATORS - 4; i++)
	// {
	// 	_actuator_optimized(i) = matrix::Vector<float, NUM_ACTUATORS - 4>(_nullspace * _lambda)(i);
	// }
	// // if(termination_type > 0 && termination_type < 5)
	// // {
	// // 	printf("_actuator_optimized no failure:\n");
	// // 	_actuator_optimized.T().print();
	// // 	_actuator_sp +=_actuator_optimized;
	// // }
	// // clipActuatorSetpoint(_actuator_sp);
	// _control_allocated =  _effectiveness * _actuator_sp;
	// _control_unallocated = (_control_sp - _control_allocated);
	// // printf("unallocated control:\n");
	// // _control_unallocated.T().print();
	// // printf("_act_sp\n");
	// // _actuator_sp.T().print();

	// printf("max:\n");
	// _actuator_max.T().print();
	// printf("min:\n");
	// _actuator_min.T().print();
	// Save into csv TODO: add a QGC bool param to control save or not save

	if (_csv_start)
	{
		// _failure_cnt++;
		// getActuatorFailure(_actuator_failure_id, _actuator_failure_val, _csv_start);
		// printf("%d, %f, %d, %d\n", _actuator_failure_id, _actuator_failure_val, _csv_start, _failure_cnt);
		cFile.open("full_tilt_fail.csv", std::ios_base::app);
		cFile << double(_control_sp(0)) << ", "<< double(_control_sp(1)) << ", "<< double(_control_sp(2)) << ", "<< double(_control_sp(3)) << ", "<< double(_control_sp(4)) << ", "<< double(_control_sp(5))<< ", " << double(_actuator_sp(0))<< ", "<< double(_actuator_sp(1))<< ", "<< double(_actuator_sp(2))<< ", "<< double(_actuator_sp(3))<< ", "<< double(_actuator_sp(4))<< ", "<< double(_actuator_sp(5))<< ", "<< double(_actuator_sp(6))<< ", "<< double(_actuator_sp(7))<< ", "<< double(_actuator_sp(8))<< ", "<< double(_actuator_sp(9))<< ", "<< double(_actuator_sp(10))<< ", "<< double(_actuator_sp(11))<< ", " <<double(_airspeed_validated.calibrated_airspeed_m_s)<<", "<<double(_vtol_vehicle_status.roll)<<", "<<double(_vtol_vehicle_status.pitch)<<", "<<double(_control_allocated(0))<<", "<<double(_control_allocated(1))<<", "<<double(_control_allocated(2))<<", "<<double(_control_allocated(3))<<", "<<double(_control_allocated(4))<<", "<<double(_control_allocated(5))<<", "<<double(_control_allocated_without_opt(0))<<", "<<double(_control_allocated_without_opt(1))<<", "<<double(_control_allocated_without_opt(2))<<", "<<double(_control_allocated_without_opt(3))<<", "<<double(_control_allocated_without_opt(4))<<", "<<double(_control_allocated_without_opt(5))<<", "<<double(_control_sp.norm())<<", "<<double(_control_allocated.norm())<<", "<<double(_control_allocated_without_opt.norm())<< ", "<< double(_actuator_sp_no_opt(0))<< ", "<< double(_actuator_sp_no_opt(1))<< ", "<< double(_actuator_sp_no_opt(2))<< ", "<< double(_actuator_sp_no_opt(3))<< ", "<< double(_actuator_sp_no_opt(4))<< ", "<< double(_actuator_sp_no_opt(5))<< ", "<< double(_actuator_sp_no_opt(6))<< ", "<< double(_actuator_sp_no_opt(7))<< ", "<< double(_actuator_sp_no_opt(8))<< ", "<< double(_actuator_sp_no_opt(9))<< ", "<< double(_actuator_sp_no_opt(10))<< ", "<< double(_actuator_sp_no_opt(11))<<", "<<double(_vtol_vehicle_status.yaw)<<", "<<int(_actuator_failure_id)<<", "<<double(_actuator_failure_val)<< ", " << double(_actoator_sp_original(0))<< ", "<< double(_actoator_sp_original(1))<< ", "<< double(_actoator_sp_original(2))<< ", "<< double(_actoator_sp_original(3))<< ", "<< double(_actoator_sp_original(4))<< ", "<< double(_actoator_sp_original(5))<< ", "<< double(_actoator_sp_original(6))<< ", "<< double(_actoator_sp_original(7))<< ", "<< double(_actoator_sp_original(8))<< ", "<< double(_actoator_sp_original(9))<< ", "<< double(_actoator_sp_original(10))<< ", "<< double(_actoator_sp_original(11))<< std::endl;
		cFile.close();
	}

	for (size_t i = 0; i < 12; i++)
	{
		_actuator_last(i) = _actuator_sp(i);
	}
	// _actuator_sp =  _actuator_trim + _mix * (_control_sp - _control_trim);
	// clipActuatorSetpoint(_actuator_sp);
	// printf("tilt: %f\n", tilt_degree);

	for (size_t i = 4; i < 8; i++)
	{
		printf("act %d = %f\n", i, _actuator_sp(i) * 180.0 / 3.14);
		_actuator_sp(i) /= 1.570796;
	}

	printf("actuator:\n");
	_actuator_sp.T().print();
	clipActuatorSetpoint(_actuator_sp);
	// printf("actuator:\n");
	// _actuator_sp.T().print();

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
void ControlAllocationPseudoInverse::_create_constraints(alglib::real_2d_array &C, alglib::integer_1d_array &ct)
{
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
						C(i, j) = _actuator_max(i) - _actuator_sp(i);
						if(i >= 4 && i < 8)
						{
							C(i, j) = _actuator_max(i) * 1.57 - _actuator_sp(i);
						}

					}
					else
					{
						C(i, j) = _actuator_max(i+1) - _actuator_sp(i+1);
						if(i >= 4 && i < 8)
						{
							C(i, j) = _actuator_max(i+1) * 1.57 - _actuator_sp(i+1);
						}
					}
					ct(i) = -1;
				}
				else
				{
					if((i%(constraint_size/2)) < (_actuator_failure_id-1))
					{
						C(i, j) = _actuator_min(i%(constraint_size/2)) - _actuator_sp(i%(constraint_size/2));
						if(i >= 4 && i < 8)
						{
							C(i, j) = _actuator_min(i%(constraint_size/2)) * 1.57 - _actuator_sp(i%(constraint_size/2));
						}
					}
					else
					{
						C(i, j) = _actuator_min((i%(constraint_size/2))+1) - _actuator_sp((i%(constraint_size/2))+1);
						if(i >= 4 && i < 8)
						{
							C(i, j) = _actuator_min((i%(constraint_size/2))+1) * 1.57 - _actuator_sp((i%(constraint_size/2))+1);
						}
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
}
int ControlAllocationPseudoInverse::_optimize_allocation_simple_cost(matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint)
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
	_create_constraints(C,ct);
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

	return rep.terminationtype;

}
int ControlAllocationPseudoInverse::_optimize_allocation_err_cost(matrix::Vector<float, NUM_ACTUATORS - 4> actuator_err)
{
	int sz = 12;
	if(_actuator_failure_id) sz--;
	matrix::Matrix<float, NUM_ACTUATORS - 4, NUM_ACTUATORS - 4> W;
	matrix::Matrix<float, NUM_ACTUATORS - 5, NUM_ACTUATORS - 5> W_failure;
	double W_motor, W_tilt, W_surface;
	if(_vtol_vehicle_status.vtol_in_rw_mode)
	{
		W_motor = 1;
		W_tilt = 5;
		W_surface = 1;
	}
	else
	{
		W_motor = 1;
		W_tilt = 1;
		W_surface = 1;
	}

	for (size_t i = 0; i < NUM_ACTUATORS - 4; i++)
	{
		if(_actuator_failure_id)
		{
			if(i < _actuator_failure_id)
			{
				if(i < 4)
				{
					W_failure(i, i) = W_motor;
				}
				else if(i >= 4 && i < 8)
				{
					W_failure(i, i) = W_tilt;
				}
				else
				{
					W_failure(i, i) = W_surface;
				}
			}
			else if (i > _actuator_failure_id)
			{
				if(i < 4)
				{
					W_failure(i - 1, i - 1) = W_motor;
				}
				else if(i >= 4 && i < 8)
				{
					W_failure(i - 1, i - 1) = W_tilt;
				}
				else
				{
					W_failure(i - 1, i - 1) = W_surface;
				}
			}
		}
		else
		{
			if(i < 4)
			{
				W(i, i) = W_motor;
			}
			else if(i >= 4 && i < 8)
			{
				W(i, i) = W_tilt;
			}
			else
			{
				W(i, i) = W_surface;
			}
		}

	}
	matrix::Matrix<float, NUM_ACTUATORS - 4, NUM_ACTUATORS - 9> N;
	matrix::Matrix<float, NUM_ACTUATORS - 5, NUM_ACTUATORS - 10> N_failure;
	for (size_t i = 0; i < sz; i++)
	{
		for (size_t j = 0; j < _null_size; j++)
		{
			if(_actuator_failure_id)
			{
				N_failure(i, j) = _nullspace_failed(i, j);
			}
			else
			{
				N(i, j) = _nullspace(i, j);
			}
		}

	}
	matrix::Matrix<float, NUM_ACTUATORS - 9, NUM_ACTUATORS - 9> A;
	matrix::Matrix<float, NUM_ACTUATORS - 10, NUM_ACTUATORS - 10> A_failure;
	alglib::real_2d_array a;
	a.setlength(_null_size, _null_size);
	if(_actuator_failure_id)
	{
		A_failure = N_failure.T() * W_failure * N_failure * 2;
		for (size_t i = 0; i < _null_size; i++)
		{
			for (size_t j = 0; j < _null_size; j++)
			{
				a(i, j) = A_failure(i, j);
			}

		}

		// printf("a_failure  = \n");
		// printAlglib(a);
	}
	else
	{
		A = N.T() * W * N * 2;
		for (size_t i = 0; i < _null_size; i++)
		{
			for (size_t j = 0; j < _null_size; j++)
			{
				a(i, j) = A_failure(i, j);
			}
		}
		// printf("a  = \n");
		// printAlglib(a);
	}

	matrix::Matrix<float, NUM_ACTUATORS - 4, 1> a_err;
	matrix::Matrix<float, NUM_ACTUATORS - 5, 1> a_err_failure;
	for (size_t i = 0; i < sz; i++)
	{
		if(_actuator_failure_id)
		{
			if(i < _actuator_failure_id)
			{
				a_err_failure(i, 0) = actuator_err(i);
			}
			else
			{
				a_err_failure(i, 0) = actuator_err(i+1);
			}

		}
		else
		{
			a_err(i, 0) = actuator_err(i);
		}

	}
	alglib::real_1d_array b;
	matrix::Matrix<float, 1, NUM_ACTUATORS - 9> linear_term;
	matrix::Matrix<float, 1, NUM_ACTUATORS - 10> linear_term_failure;
	b.setlength(_null_size);
	if(_actuator_failure_id)
	{
		linear_term_failure = a_err_failure.T() * W_failure * N_failure;
		for (size_t i = 0; i < _null_size; i++)
		{
			b(i) = linear_term_failure(0, i);
		}

		// printf("b_failure  = \n");
		// printAlglib(b);
	}
	else
	{
		linear_term = a_err.T() * W * N;
		for (size_t i = 0; i < _null_size; i++)
		{
			b(i) = linear_term_failure(0, i);
		}
		// printf("b  = \n");
		// printAlglib(b);
	}

	// printf("nullsize = %d\n", _null_size);

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
		s(i) = 1 / sqrt(4 * W_surface + 4 * W_tilt + 4 * W_motor);
	}

	alglib::real_2d_array C;
	alglib::integer_1d_array ct;
	_create_constraints(C,ct);
	// printf("null:\n");
	// _nullspace_failed.print();
	// printf("constraints2:\n");
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
	alglib::minqpsetscale(state, s);
	// alglib::minqpsetscaleautodiag(state);

	alglib::minqpsetalgobleic(state, 0.2, 0.2, 0.2, 0);
	// alglib::minqpsetalgoquickqp(state, 0.001, 0.001, 0.001, 0, true);
	alglib::minqpoptimize(state);
	alglib::minqpresults(state, x, rep);

	if(rep.inneriterationscount > mx) mx = rep.inneriterationscount;
	printf("solution_err_cost_func = \n");
	printAlglib(x);
	printf("opt report 2: \ninner cnt: %d\nouter cnt:%d\ntermination type:%d\n", rep.inneriterationscount, rep.outeriterationscount, rep.terminationtype);
	printf("max iter = %d\n", mx);
	_lambda_sol = x;

	return rep.terminationtype;
}
