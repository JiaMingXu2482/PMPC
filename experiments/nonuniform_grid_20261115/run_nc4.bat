@echo off
cd /d "D:\OneDrive - mail.scut.edu.cn\Thesis\Code\Controller\MAIN\experiments\nonuniform_grid_20261115"
matlab -batch "worker_boot(pwd); run_nu('uniform',3,70,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc70_nc_uni')); disp('1');" > fair_n1.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('nu',3,70,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc70_nc_nu')); disp('2');" > fair_n2.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('uniform',3,80,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc80_nc_uni')); disp('3');" > fair_n3.log 2>&1
matlab -batch "worker_boot(pwd); run_nu('nu',3,80,struct('stopTime',15,'tau_d',1e-4,'prefix','fair_dlc80_nc_nu')); disp('4');" > fair_n4.log 2>&1
