@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\full_actuator_audit_20261116\logging"
matlab -batch "addpath(pwd); try, replay_bench2('sla80'); catch ME, f=fopen('replay_err_sla.txt','w','n','UTF-8'); fprintf(f,'%s\n',getReport(ME,'extended','hyperlinks','off')); fclose(f); disp('REPLAY_FAILED'); end" > rep_sla.log 2>&1
echo EXIT=%ERRORLEVEL% >> rep_sla.log
