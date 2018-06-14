%% A Kalman filter for the bike project
% A Kalman filter algorithm that works with 1500-point data series
% intended for the self-balancing bicycle project
% 
% @author Gavin Fielder
% @date 12/6/2017
% 
% Dependencies: the following files are needed in the working directory:
%   (function) measurement_noise_covariance_matrix_R.m
%       This can be replaced by MATLAB's cov() function; I implemented the 
%       covariance function myself in order to later port it to C code.
%   (script) getData.m
%       This script loads data from the file specified by 'filename' and
%       separates the data into data series.
%       getData.m will need to be rewritten any time the data storage
%       format changes.
%   (data) baseline.mat
%       This data holds the baseline sensor readings. This is needed to 
%       determine the measurement noise covariance matrix R.
%   (data) Q.mat
%       This matrix holds the matrix 'Q_in' and is used to specify Q.
%   Of course, you also need the file containing the data to filter.
%
% Instructions for use:
%       1. Enter the file name and the name of the data series
%          in the first section below. 
%       2. Run the script.
%
% Known issues:
%       *  It's not perfect. Gyro readings especially are still noisy even
%          with the kalman filter, and I don't know why. Changing Q
%          can reduce more noise (see discussion below), but introduces a
%          a time lag in the state estimation. 
%
% Notes on this implementation:
%       *  My predictive matrix uses the first and second derivatives
%          making this kalman filter three-dimensional
%

%% User select data series
file_in = 'fusdat_pickup_play2_thenreturn.mat' % also output to console
lookingAt = 'accel_x' % also output to console


%%

% % clear the workspace - This section is for Q estimation
% if ~(exist('saved_Q','var'))
%     i = 1;
%     saved_Q = zeros(2,2,1);
% end
% % clc; 
% clearvars -except file_in lookingAt i saved_Q %Q_in cova_saved;

% Clear the workspace - use this for normal operation (not Q estimation)
clc; clearvars -except file_in lookingAt;

%% Load the Process Noise covariance matrix Q
% "In Kalman filtering the "process noise" represents the idea/feature 
%  that the state of the system changes over time, but we do not know 
%  the exact details of when/how those changes occur, and thus we need 
%  to model them as a random process." 
%  from: https://stackoverflow.com/questions/19537884/explain-process-noise-terminology-in-kalman-filter
%
% In general, the noise process covariance matrix is difficult to find.
% I believe I accidentally came up with a Q that works. My method is as
% follows:
%   1. Run the Kalman filter when Q = 0;
%   2. Average the resulting state covariance matrices (cova) for each
%      time step into a single matrix. Set Q_in to be this matrix.
% The Q.mat I've included in this release is actually an average of all
% covariance matrices over all time steps from the 10 accel/gyro
% data series from the fusdat_pickup_play#_thenreturn.mat files
%
% The criteria by which I saw "this seems to work" is that it appears to be
% a good approximation of the actual state--it goes through the center of
% the data points. Choosing different values of Q based on the state
% covariance matrices produced a time lag, which I felt was likely worse,
% because a time lag would constitute a systemic error. 
% However, apart from the time lag, these matrices produced a much better
% noise reduction, so if the noise reduction at the cost of having a time
% lag in the data is a tradeoff we want to make, I can tune Q in this way.
%
% There may be a better Q. There may be a better way to find Q.
% I do not know how to find it. I don't even know if this method of 
% selecting Q is sound. It just seems to work.
%

load(sprintf('Q_%s.mat',lookingAt)); % load Q from the appropriate file
% Note the above file contains 'Q_in', not 'Q'. When 'Q_in' is set, 
% Q will be set to Q_in. Otherwise, Q will be zero.


%% Calculate measurement covariance matrix R from the baseline data 
% Load the baseline data for determining measurement noise covariance R
%
% In practice, we should be able to measure R and then pre-load it through
% the STM's software. 
%

filename = 'baseline.mat';
getData;

% construct first and second derivatives of data
eval(sprintf('data(:,1) = %s;',lookingAt)); % load the user-selected data series
data(2:end,2) = data(2:end,1) - data(1:end-1,1); % first derivative
% Note this was the forward derivative too but it shouldn't matter for the
% baseline

% Now we need to construct R, the measurement noise covariance matrix
% R = cov(data); % use this line to switch to MATLAB's internal cov() function
R = measurement_noise_covariance_matrix_R(data)

%% Load the User-selected Data

filename = file_in; % user defined
run getData; 

% construct first derivative of data
eval(sprintf('data(:,1) = %s;',lookingAt)); % load the user-selected data series
data(2:end,2) = data(2:end,1) - data(1:end-1,1); % first derivative

%% Plot raw data
t = (1:1500)';
figure(6); clf;
plot(t, data(:,1),'k-','MarkerSize',3); hold on; grid on;

%% Set up Kalman Filter
N = length(data);
x = zeros(N+1,2);           % col 1: state variable
x_predicted = zeros(N+1,2); % col 2: first derivative
% Note matrices are separated on the third dim rather than the first
gain = zeros(2,2,N+1);      % kalman gain - also a square matrix
cova = zeros(2,2,N+1);      % state covariance matrices at each step
cova_predicted = zeros(2,2,N+1); % predicted covariance matrices
A = [   1   1   ;       
        0   1   ];
H = [   1   0   ;          % Relational matrix of sensor and state, H
        0   1   ];         % This is identity because we just want to
                           % know the "true reading" of the sensor
    
if (exist('Q_in','var')) % If there's a 'Q_in', then use that.
    Q = Q_in 
else                      % If not, set Q to zero.
    Q = [   0   0   ;           % Process noise covariance
            0   0   ]                 % no idea what this is or should be
                                      % UNC says this'll have to be 'tuned'
end                                   % but I'll set it to zero since it's
                                      % used as an offset.
    
% Set initial values
x(1,1) = data(1);
x(1,2) = 0;
cova(:,:,1) = [ 1   0   ;       % I don't actually know what this should be
                0   1   ];      % but I figure identity is a good guess
                                % since bilbo's 1-dimensional uses 1
% We don't need to set initial gain because it's not calculated as 
% a difference equation like these others
 
%% Begin Kalman filtering

for k = 2:N % I feel like this should be N+1, but that doesn't work in this algorithm
            % I'll need to go through this sometime and verify the offsets
            % aren't causing any issues.
   % Begin predictive time update
     % Calculate prediction of upcoming state based on previous state
     x_predicted(k,:) = (A*(x(k-1,:)'))';
     % Calculated predicted covariance matrix
     cova_predicted(:,:,k) = A*cova(:,:,k-1)*(A') + Q; 
    
   % Begin measurement update
     % Calculate the Kalman Gain -- remember currently H is identity
     Pe = cova_predicted(:,:,k);  % for notational simplicity
     % gain(:,:,k) = Pe*(H')*(inv(H*Pe*(H') + R));
     gain(:,:,k) = Pe/(Pe + R); % assumes H is identity
     % Calculate new filtered state estimate
     xe = x_predicted(k,:)';        % for notational simplicity
     z = data(k,:)';              % for notational simplicity
     K = gain(:,:,k);             % for notational simplicity
     x(k,:) = xe + K*(z - H*xe);
     % Now update the error covariance
     cova(:,:,k) = (eye(2) - K*H)*cova_predicted(:,:,k);

   % Ready for next iteration
end

%% Plot the Kalman Filter results in red
plot(t+1,x(2:end,1),'-r','LineWidth',2);

%%
% Save a copy of the covariance matrices. 

% cova_saved = cova;

%%
% 
% % Select Q from the saved covariance matrices
% 
% % Q_in = cova_saved(:,:,500);
% tmpQ = mean(cova_saved(:,:,2:1499),3);
% 
% % save this Q to the saved_Q array
% saved_Q(:,:,i) = tmpQ;
% i = i + 1;
% 
% % output parameters of the saved Q
% saved_Q(:,:,i-1)

%% Import C ouput

c_output = importdata("C:\Users\gavin\Documents\Self-Balancing Bicycle Project-20171110T225134Z-001\Kalman Filter in C\CodeBlocks_Testing\kalman_filter_test\c_output.txt");

plot(t,c_output,'-b');




