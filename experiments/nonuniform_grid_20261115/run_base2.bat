@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Baseline"
matlab -batch "worker_boot(pwd); run_baseline('dlc',75,struct('stopTime',15,'prefix','fair_bl_dlc75')); disp('B1');" > fair_b1.log 2>&1
echo EXIT=%ERRORLEVEL% >> fair_b1.log
matlab -batch "worker_boot(pwd); run_baseline('slalom',80,struct('stopTime',8,'prefix','fair_bl_sla80')); disp('B2');" > fair_b2.log 2>&1
echo EXIT=%ERRORLEVEL% >> fair_b2.log
