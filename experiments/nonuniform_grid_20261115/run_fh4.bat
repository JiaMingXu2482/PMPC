@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu_fh('nocomp','uniform',3,struct('stopTime',8,'cargo',170)); disp('F1');" > fh1.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_fh('nocomp','nu',3,struct('stopTime',8,'cargo',170)); disp('F2');" > fh2.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_fh('comp25','uniform',3,struct('stopTime',8,'cargo',170)); disp('F3');" > fh3.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_fh('comp25','nu',3,struct('stopTime',8,'cargo',170)); disp('F4');" > fh4.log 2>&1
