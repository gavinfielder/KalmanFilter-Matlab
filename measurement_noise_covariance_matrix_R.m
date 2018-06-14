function [ R ] = measurement_noise_covariance_matrix_R( data )
%measurement_noise_covariance_matrix_R  Computes the covariance matrix of
    % an N x 3 data matrix
    
    % Simplify notation
    x = data(:,1);
    y = data(:,2);
%     z = data(:,3);
    N = size(data,1);
    
    xm = mean(x);
    ym = mean(y);
%     zm = mean(z);
    
    a11 = sum((x - xm).*(x - xm)) / (N - 1);
    a21 = sum((x - xm).*(y - ym)) / (N - 1);
    a22 = sum((y - ym).*(y - ym)) / (N - 1);
%     a31 = sum((x - xm).*(z - zm)) / (N - 1);
%     a32 = sum((z - zm).*(y - ym)) / (N - 1);
%     a33 = sum((z - zm).*(z - zm)) / (N - 1);
    
%     R = [   a11     a21     a31     ;
%             a21     a22     a32     ;
%             a31     a32     a33     ];

      R = [   a11     a21    ;
              a21     a22    ];

      end

      
      
      
      
      
      