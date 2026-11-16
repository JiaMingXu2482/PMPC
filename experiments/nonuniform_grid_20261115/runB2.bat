@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('nu',3,70,struct('stopTime',3)); disp('RUN_B2_DONE');" > runB2.log 2>&1
echo EXIT=%ERRORLEVEL% >> runB2.log
