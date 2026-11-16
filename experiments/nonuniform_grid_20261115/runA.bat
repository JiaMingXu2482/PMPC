@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,70,struct('stopTime',15)); disp('RUN_A_DONE');" > runA.log 2>&1
echo EXIT=%ERRORLEVEL% >> runA.log
