@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,75,struct('stopTime',15,'prefix','dec_u75')); disp('U');" > dec_u75.log 2>&1
echo EXIT=%ERRORLEVEL% >> dec_u75.log
matlab -batch "worker_boot(pwd); run_nu('nu',3,75,struct('stopTime',15,'prefix','dec_nu75')); disp('N');" > dec_nu75.log 2>&1
echo EXIT=%ERRORLEVEL% >> dec_nu75.log
