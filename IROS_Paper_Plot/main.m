% close all
clear all
clc

% file = csvread('test_data/motor_failure_1/motor_failure_1.csv', 2, 0);
% file = csvread('test_data/tuning_tests/tuning_tests_with_sub.csv', 2, 0);
% file = csvread('test_data/tilt_failure_1/tilt_failure_1.csv', 2, 0);
% file = csvread('test_data/motor_failure_fw_1/motor_failure_fw_1.csv', 2, 0);
file = csvread('/home/mohammad/workspace/px4_tilt_test/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/new_motor.csv', 2, 0);
% file = csvread('test_data/final_tests/motor_fail_to_0-3/with_opt.csv', 2, 0);
% file = csvread('test_data/final_tests/no_knowledge_is_better!/opt_motor.csv', 2, 0);
% file = csvread('test_data/final_tests/no_knowledge_is_better!/pid_motor.csv', 2, 0);
% file = csvread('test_data/final_tests/no_knowledge_is_better!/opt_tilt.csv', 2, 0);
% file = csvread('test_data/final_tests/no_knowledge_is_better!/pid_tilt.csv', 2, 0);

control_sp = file(:,1:6);
actuator_sp = file(:, 7:18);
airspeed = file(:, 19);
roll = file(:, 20);
pitch = file(:, 21);
control_allocated = file(:, 22:27);
control_allocated_no_opt = file(:, 28:33);
norm_control_sp = file(:, 34);
norm_control_allocated = file(:, 35);
norm_control_allocated_no_opt = file(:, 36);
actuator_sp_no_opt = file(:, 37:48);
yaw = file(:, 49);
fail_id = file(:, 50);
fail_val = file(:, 51);
actuator_sp_original = file(:, 52:63);

fig_num = 0;
%% plot norm c_sp vs c_alloc
% fig_num = fig_num + 1;
% figure(fig_num)
% plot(norm_control_sp)
% 
% plot(norm_control_allocated - norm_control_sp)
% hold on
% plot(norm_control_allocated_no_opt - norm_control_sp)
% legend('alloc','alloc no opt')
%% plot acts
fig_num = fig_num + 1;
figure()
s = ["motor", "tilt", "control surface"];
for i = 1:12
    subplot(3, 4, i)
    plot(actuator_sp(:,i))
    hold on
    plot(actuator_sp_no_opt(:,i))
%     plot(actuator_sp_original(:,i))
%     if(ceil(i/4) == 2)
%         ylim([0.9, 1.1])
%     end
    legend('act_sp','no opt','original')
    ylim([-1.0, 1.0])
    xlim([0, 4400])
    title(s(ceil(i/4)) + ' '+int2str(rem(i-1, 4)))
end