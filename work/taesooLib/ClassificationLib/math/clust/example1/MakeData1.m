N=500;

R1 = [ 1, 0.1; 0.1, 1];
mu1 = [2, 2]';

R2 = [ 1, -0.1; -0.1, 1];
mu2 = [-2, -2]';

R3 = [ 1, 0.2; 0.2, 0.5];
mu3 = [5.5, 2]';

Pi = [0.4, 0.4, 0.2];

% Generate Data from Gaussian mixture model
x = GMM(N,Pi,mu1,mu2,mu3,R1,R2,R3);

plot(x(1,:),x(2,:),'o');
title('Scatter Plot of Multimodal Data')
xlabel('first component')
ylabel('second component')

x = x';
save data x /ascii

