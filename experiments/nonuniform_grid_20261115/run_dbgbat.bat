@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\Baseline"
matlab -batch "worker_boot(pwd); run_baseline('slalom',80,struct('stopTime',8,'prefix','dbg_bl_sla_md0','md_frozen_on',false)); disp('D');" > dbg_md0.log 2>&1
echo EXIT=%ERRORLEVEL% >> dbg_md0.log
