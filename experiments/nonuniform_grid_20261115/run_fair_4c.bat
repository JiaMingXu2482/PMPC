@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,75,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc75_nc_uni')); disp('D1');" > fair_d1.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('nu',3,75,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc75_nc_nu')); disp('D2');" > fair_d2.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('uniform',3,75,struct('stopTime',15,'prefix','fair_dlc75_c_uni')); disp('D3');" > fair_d3.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('nu',3,75,struct('stopTime',15,'prefix','fair_dlc75_c_nu')); disp('D4');" > fair_d4.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_sla('uniform',3,80,struct('stopTime',8,'tau_d',1e-4,'prefix','fair_sla80_nc_uni')); disp('S1');" > fair_s1.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_sla('nu',3,80,struct('stopTime',8,'tau_d',1e-4,'prefix','fair_sla80_nc_nu')); disp('S2');" > fair_s2.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_sla('uniform',3,80,struct('stopTime',8,'prefix','fair_sla80_c_uni')); disp('S3');" > fair_s3.log 2>&1
matlab -batch "worker_boot(pwd); run_nu_sla('nu',3,80,struct('stopTime',8,'prefix','fair_sla80_c_nu')); disp('S4');" > fair_s4.log 2>&1
