@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu_sla('uniform',3,80,struct('stopTime',8)); disp('DONE_sla_u');" > run_sla_u.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_sla_u.log
matlab -batch "worker_boot(pwd); run_nu_sla('nu',3,80,struct('stopTime',8)); disp('DONE_sla_n');" > run_sla_n.log 2>&1
echo EXIT=%ERRORLEVEL% >> run_sla_n.log
