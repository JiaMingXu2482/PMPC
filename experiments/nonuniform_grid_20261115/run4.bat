@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,75,struct('stopTime',15)); disp('DONE_u75');" > run_u75.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_u75.log
matlab -batch "worker_boot(pwd); run_nu('nu',3,75,struct('stopTime',15)); disp('DONE_n75');" > run_n75.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_n75.log
matlab -batch "worker_boot(pwd); run_nu('uniform',3,80,struct('stopTime',15)); disp('DONE_u80');" > run_u80.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_u80.log
matlab -batch "worker_boot(pwd); run_nu('nu',3,80,struct('stopTime',15)); disp('DONE_n80');" > run_n80.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_n80.log
