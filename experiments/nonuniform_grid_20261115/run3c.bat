@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',1,70,struct('stopTime',15)); disp('DONE_c1_70');" > run_c1_70.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c1_70.log
matlab -batch "worker_boot(pwd); run_nu('uniform',1,75,struct('stopTime',15)); disp('DONE_c1_75');" > run_c1_75.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c1_75.log
matlab -batch "worker_boot(pwd); run_nu('uniform',1,80,struct('stopTime',15)); disp('DONE_c1_80');" > run_c1_80.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c1_80.log
matlab -batch "worker_boot(pwd); run_nu('uniform',2,70,struct('stopTime',15)); disp('DONE_c2_70');" > run_c2_70.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c2_70.log
matlab -batch "worker_boot(pwd); run_nu('uniform',2,75,struct('stopTime',15)); disp('DONE_c2_75');" > run_c2_75.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c2_75.log
matlab -batch "worker_boot(pwd); run_nu('uniform',2,80,struct('stopTime',15)); disp('DONE_c2_80');" > run_c2_80.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_c2_80.log
