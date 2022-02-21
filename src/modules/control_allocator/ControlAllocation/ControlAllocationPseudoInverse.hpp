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
 * It computes the pseudo-inverse of the effectiveness matrix
 * Actuator saturation is handled by simple clipping, do not
 * expect good performance in case of actuator saturation.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/vehicle_status.h>

class ControlAllocationPseudoInverse: public ControlAllocation
{
public:
	ControlAllocationPseudoInverse() = default;
	virtual ~ControlAllocationPseudoInverse() = default;

	virtual void allocate() override;
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim, int num_actuators) override;


private:
	vtol_vehicle_status_s _vtol_vehicle_status {};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
	int print_cnt = 0;
	alglib::real_1d_array _lambda_sol;
	alglib::real_1d_array _last_lambda_sol;
	bool _last_lambda_init = false;
	void _optimize_sample();
	void _optimize_allocation(matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> linear_constraint, matrix::Vector<float, NUM_ACTUATORS - 4> actuator_opt);

protected:
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;
	// ADDED
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix_unknown;
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix_known;
	// ADDED
	bool _mix_update_needed{false};

	/**
	 * Recalculate pseudo inverse if required.
	 *
	 */
	void updatePseudoInverse();
};
