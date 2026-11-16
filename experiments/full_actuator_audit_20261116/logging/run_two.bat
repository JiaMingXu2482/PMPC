@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\full_actuator_audit_20261116\logging"
matlab -batch "addpath(pwd); run_log_case('dlc75',75,3,struct('stopTime',15,'tau_d',1e-4)); disp('DLC_DONE');" > run_dlc75.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_dlc75.log
matlab -batch "addpath(pwd); make_log_model('sla80'); disp('MODEL_SLA_DONE');" > mk_sla.log 2>&1
echo EXIT=%ERRORLEVEL% >> mk_sla.log
matlab -batch "addpath(pwd); run_log_case('sla80',80,3,struct('stopTime',8,'tau_d',1e-4)); disp('SLA_DONE');" > run_sla80.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_sla80.log
