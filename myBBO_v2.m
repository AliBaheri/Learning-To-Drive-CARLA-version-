
tic
OVERALL = [];
cost_next = [];
num_batch = 5;
simulation_parameters_partitionedCB;
boundss=[-5*pi/180 10*pi/180];
%%% initial guess for plant parameters
bounds = [0.0001 0.0025,0.001 0.0052];
A_H = rand(5,1)*(bounds(2)-bounds(1)) + bounds(1);  
x_cm = rand(5,1)*(bounds(4)-bounds(3)) + bounds(3);
initial_guess = [A_H x_cm];
%%% evaluate performance for plant parameters
cost = [];
for ii = 1:length(initial_guess)
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant16','Value', num2str(initial_guess(ii,1)));
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant17','Value', num2str(initial_guess(ii,2)));
    simout_guess = sim('v39_kinematictether_variable_plant_partitioned', 'SimulationMode', 'normal');
    J_plant = simout_guess.get('cost_parameters');
    cost(end+1,:) = J_plant.Data(end);
end
%%% convert intial guess to python version
A_H = py.numpy.reshape(A_H(:)',size(A_H),'F');
x_cm= py.numpy.reshape(x_cm(:)',size(x_cm),'F');
initial_guess = py.numpy.reshape(initial_guess(:)',size(initial_guess),'F');
cost = py.numpy.reshape(cost(:)',size(cost),'F');
%%% main loop - outer loop: Batch Bayesian Optimization
new_cost = [];

for jj = 1:10
      
    next_Batchplant = py.Batch5.BAYESOPT2(A_H,x_cm,cost);
    next_Batchplant = double(py.array.array('d',py.numpy.nditer(next_Batchplant)));
    next_Batchplant = reshape(next_Batchplant,[2 5])'; 
    for kk = 1:length(next_Batchplant)
        set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant16','Value', num2str(next_Batchplant(kk,1)));
        set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant17','Value', num2str(next_Batchplant(kk,2)));
        simout = sim('v39_kinematictether_variable_plant_partitioned', 'SimulationMode', 'normal');
        J = simout.get('cost_parameters');
        new_cost(kk) = J.Data(end);
    end
    
    [min_new_cost,index] = min(new_cost);
    
    new_cost = py.numpy.reshape(new_cost(:)',size(new_cost),'F');
    cost = py.append.APPEND(cost,new_cost);
    
    next_A_H = py.numpy.reshape(next_Batchplant(:,1)',size(next_Batchplant(:,1)),'F');
    next_x_cm = py.numpy.reshape(next_Batchplant(:,2)',size(next_Batchplant(:,2)),'F');
    
    A_H = py.append.APPEND(A_H,next_A_H);
    x_cm = py.append.APPEND(x_cm,next_x_cm);
   
    new_cost = double(py.array.array('d',py.numpy.nditer(new_cost)));
    new_cost = new_cost';

%%% inner loop: Bayesian Optimization for controller parameter
theta_opt = rand(4,1)*(boundss(2)-boundss(1)) + boundss(1);

for jdx = 1:length(theta_opt)
       set_param('v39_kinematictether_variable_plant_partitioned/controller/theta_des_BO','Value', num2str(theta_opt(jdx)));
       simout_guess(jdx) = sim('v39_kinematictether_variable_plant_partitioned', 'SimulationMode', 'normal');
       J_guess = simout_guess(jdx).get('cost_parameters');
       data_val = J_guess.Data;
       k(jdx,1) = data_val(end);
end 
%%% initial guess in inner loop for control parameter
x = [theta_opt(1) theta_opt(2) theta_opt(3) theta_opt(4)]';
%y = [-k(1) -k(2) -k(3) -k(4)]';
y = [k(1) k(2) k(3) k(4)]';

% x = [theta_opt(1) theta_opt(2)]';
% y = [-k(1) -k(2)]';

x = py.numpy.reshape(x(:)',size(x),'F');
y = py.numpy.reshape(y(:)',size(y),'F');


 for zz = 1:10
    % run full Bayesian Optimization in inner loop
    %theta_des_BO = BayesOpt(x,y,boundss);
    theta_des_BO = py.BAYESOPT1D.BAYESOPT1D(x,y);
    theta_des_BO = double(py.array.array('d',py.numpy.nditer(theta_des_BO)));
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant16','Value', num2str(next_Batchplant(index,1)));
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant17','Value', num2str(next_Batchplant(index,2)));
    set_param('v39_kinematictether_variable_plant_partitioned/controller/theta_des_BO','Value', num2str(theta_des_BO));
    simout_aa  = sim('v39_kinematictether_variable_plant_partitioned','SimulationMode','normal');  
    JJ = simout_aa.get('cost_parameters');
    s = JJ.Data(end);
     
    % append the best solution and associated cost to initial guess
    
    x = py.append.APPEND(x,theta_des_BO);
    y = py.append.APPEND(y,s);

%     y = [y' -s]';
%     x = [x' theta_des_BO]';
 end
     
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant16','Value', num2str(next_Batchplant(index,1)));
    set_param('v39_kinematictether_variable_plant_partitioned/plant/Constant17','Value', num2str(next_Batchplant(index,2)));
    set_param('v39_kinematictether_variable_plant_partitioned/controller/theta_des_BO','Value', num2str(theta_des_BO));
    simout_b = sim('v39_kinematictether_variable_plant_partitioned', 'SimulationMode', 'normal');
    J_ove = simout_b.get('cost_parameters');
    SS = J_ove.Data(end);
    OVERALL(end+1,:) = SS;
end
toc

A_H = double(py.array.array('d',py.numpy.nditer(A_H)));
stairs(A_H);
 
x_cm = double(py.array.array('d',py.numpy.nditer(x_cm)));
stairs(x_cm);

x = double(py.array.array('d',py.numpy.nditer(x)));
stairs(x);


cost = double(py.array.array('d',py.numpy.nditer(cost)));
stairs(cost)
 
filename = 'BB0_5batch_element.mat';
save(filename);

 