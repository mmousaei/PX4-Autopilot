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
 * @file ControlAllocation.hpp
 *
 * Interface for Control Allocation Algorithms
 *
 * Implementers of this interface are expected to update the members
 * of this base class in the `allocate` method.
 *
 * Example usage:
 * ```
 * [...]
 * // Initialization
 * ControlAllocationMethodImpl alloc();
 * alloc.setEffectivenessMatrix(effectiveness, actuator_trim);
 * alloc.setActuatorMin(actuator_min);
 * alloc.setActuatorMin(actuator_max);
 *
 * while (1) {
 * 	[...]
 *
 * 	// Set control setpoint, allocate actuator setpoint, retrieve actuator setpoint
 * 	alloc.setControlSetpoint(control_sp);
 * 	alloc.allocate();
 * 	actuator_sp = alloc.getActuatorSetpoint();
 *
 * 	// Check if the control setpoint was fully allocated
 *	unallocated_control = control_sp - alloc.getAllocatedControl()
 *
 *	[...]
 * }
 * ```
 *
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "alglib-cpp/src/stdafx.h"
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_actuator_setpoint.h>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <fstream>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include "alglib-cpp/src/optimization.h"
#include "alglib-cpp/src/ap.h"
#include "alglib-cpp/src/linalg.h"
#include "alglib-cpp/src/alglibmisc.h"
#include "alglib-cpp/src/alglibinternal.h"
#include "alglib-cpp/src/solvers.h"
#include "alglib-cpp/src/stdafx.h"

class ControlAllocation
{
public:
	ControlAllocation() = default;
	virtual ~ControlAllocation() = default;

	static constexpr uint8_t NUM_ACTUATORS = 16;
	static constexpr uint8_t NUM_AXES = 6;
	static constexpr uint8_t NUM_KNOWN = 4;
	static constexpr uint8_t NUM_UNKNOWN = 8;


	typedef matrix::Vector<float, NUM_ACTUATORS> ActuatorVector;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};
	// FILE * cFile;
	std::ofstream cFile;

	/**
	 * Allocate control setpoint to actuators
	 *
	 * @param control_setpoint  Desired control setpoint vector (input)
	 */
	virtual void allocate() = 0;

	/**
	 * Set the control effectiveness matrix
	 *
	 * @param B Effectiveness matrix
	 */
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim, int num_actuators);

	/**
	 * Get the allocated actuator vector
	 *
	 * @return Actuator vector
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorSetpoint() const { return _actuator_sp; }

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	void setControlSetpoint(const matrix::Vector<float, NUM_AXES> &control) { _control_sp = control; }

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getControlSetpoint() const { return _control_sp; }

	/**
	 * Get the allocated control vector
	 *
	 * @return Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getAllocatedControl() const { return _control_allocated; }

	/**
	 * Get the control effectiveness matrix
	 *
	 * @return Effectiveness matrix
	 */
	const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &getEffectivenessMatrix() const { return _effectiveness; }

	/**
	 * Set the minimum actuator values
	 *
	 * @param actuator_min Minimum actuator values
	 */
	void setActuatorMin(const matrix::Vector<float, NUM_ACTUATORS> &actuator_min) { _actuator_min = actuator_min; }

	/**
	 * Get the minimum actuator values
	 *
	 * @return Minimum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMin() const { return _actuator_min; }

	/**
	 * Set the maximum actuator values
	 *
	 * @param actuator_max Maximum actuator values
	 */
	void setActuatorMax(const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) { _actuator_max = actuator_max; }

	/**
	 * Get the maximum actuator values
	 *
	 * @return Maximum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMax() const { return _actuator_max; }

	/**
	 * Set the current actuator setpoint.
	 *
	 * Use this when a new allocation method is started to initialize it properly.
	 * In most cases, it is not needed to call this method before `allocate()`.
	 * Indeed the previous actuator setpoint is expected to be stored during calls to `allocate()`.
	 *
	 * @param actuator_sp Actuator setpoint
	 */
	void setActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);

	/**
	 * Clip the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [min; max]
	 *
	 * @param actuator Actuator vector to clip
	 */
	void clipActuatorSetpoint(matrix::Vector<float, NUM_ACTUATORS> &actuator) const;

	/**
	 * Normalize the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [-1; +1]
	 *
	 * @param actuator Actuator vector to normalize
	 *
	 * @return Clipped actuator setpoint
	 */
	matrix::Vector<float, NUM_ACTUATORS> normalizeActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator)
	const;

		/**
	 * Author: Mohammad
	 * Find the nullspace of the matrix
	 *
	 * @param m Input matrix
	 *
	 * @param nullspace Output nullspace of input m
	 * @param nullsize Output nullspace sice of input m
	 */
	void getNullSpace(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &m, matrix::Matrix<float, NUM_ACTUATORS - 4, NUM_ACTUATORS - 4> &nullspace, int &nullsize);

	/**
	 * Author: Mohammad
	 * Convert from px4 matrix to alglib matrix
	 *
	 * @param m Input matrix
	 *
	 * @return Converted matrix
	 */
	alglib::real_2d_array matrixToAlglib(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &m, int size);
	alglib::real_1d_array matrixToAlglib(const matrix::Matrix<float, 1,  NUM_ACTUATORS - 4> &m);
	alglib::real_2d_array matrixToAlglib(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS - 5> &m);

	/**
	 * Author: Mohammad
	 * Convert from alglib matrix to px4 matrix
	 *
	 * @param m Input matrix
	 *
	 * @return Converted matrix
	 */
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> alglibToMatrix(const alglib::real_2d_array &m);


	void printAlglib(const alglib::real_2d_array &a);
	void printAlglib(const alglib::real_1d_array &a);

	bool failed = false;
	virtual void updateParameters() {}

	int numConfiguredActuators() const { return _num_actuators; }
	void getActuatorFailure(int failure_id, float failure_val) {
		_actuator_failure_id = failure_id;
		_actuator_failure_val = failure_val;
		if(_actuator_failure_id != 0) failed = true;
	}

	airspeed_validated_s 				_airspeed_validated{};
	vtol_vehicle_status_s _vtol_vehicle_status {};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};

protected:
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness;  //< Effectiveness matrix
	matrix::Matrix<float, NUM_ACTUATORS - 4, NUM_ACTUATORS - 4> _nullspace;  //< Effectiveness matrix
	matrix::Matrix<float, NUM_ACTUATORS - 5, NUM_ACTUATORS - 4> _nullspace_failed;  //< Effectiveness matrix
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim; 	//< Neutral actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_min; 	//< Minimum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_max; 	//< Maximum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;  	//< Actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp_uk;  	//< Actuator setpoint
	matrix::Vector<float, NUM_AXES> _control_sp;   		//< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_allocated;  	//< Allocated control
	matrix::Vector<float, NUM_AXES> _control_trim;  	//< Control at trim actuator values
	// ADDED
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness_unknown;  //< Effectiveness matrix
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness_known;  //< Effectiveness matrix
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim_unknown;        //< Neutral actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim_known;  //< Neutral actuator values
	matrix::Vector<float, NUM_AXES> _control_trim_unknown;          //< Control at trim actuator values
	matrix::Vector<float, NUM_AXES> _control_trim_known;    //< Control at trim actuator values
	matrix::Vector<float, NUM_AXES> _control_known_sp;              //< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_unknown_sp;            //< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_allocated_known;       //< Allocated control
	matrix::Vector<float, NUM_AXES> _control_allocated_unknown;     //< Allocated control
	matrix::Vector<float, NUM_ACTUATORS> _actuator_unknown_sp;  	//< Actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _actuator_known_sp;  	//< Actuator setpoint
	matrix::Vector<float, NUM_ACTUATORS> _actuator_unallocated_sp;  	//< Actuator setpoint
	std::vector<int> known_ind{};

	int _null_size;
	int _actuator_failure_id;	//failed actuator id given from QGC
	float _actuator_failure_val;	//failed actuator value given from QGC

	// float known_val[NUM_KNOWN];

	// ADDED
	int _num_actuators{0};
};
