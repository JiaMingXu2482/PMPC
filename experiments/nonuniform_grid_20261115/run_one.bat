@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,75,struct('stopTime',3,'prefix','dbg_u3')); disp('OK');" > dbg_u3.log 2>&1
echo EXIT=%ERRORLEVEL% >> dbg_u3.log
