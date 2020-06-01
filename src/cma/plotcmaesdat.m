function plotcmaesdat(figNb, filenameprefix, filenameextension, objectvarname)
% PLOTCMAESDAT;
% PLOTCMAES(FIGURENUMBER_iBEGIN_iEND, FILENAMEPREFIX, FILENAMEEXTENSION, OBJECTVARNAME);
%   plots output from CMA-ES, e.g. cmaes.m, Java class CMAEvolutionStrategy... 
%   mod(figNb,100)==1 plots versus iterations. 
%
% PLOTCMAES([11 300]) plots versus iteration, from iteration 300. 
% PLOTCMAES([10 150 800]) plots versus function evaluations, between iteration 150 and 800. 
%
% Files [FILENAMEPREFIX name FILENAMEEXTENSION] are used, where 
%   name = axlen, OBJECTVARNAME (xmean|xrecentbest), fit, or stddev.
%

  if nargin < 1 | isempty(figNb)
    figNb = 325;
  end
  if nargin < 2 || isempty(filenameprefix)
    filenameprefix = 'outcmaes';
  end
  if nargin < 3 || isempty(filenameextension)
    filenameextension = '.dat';
  end
  if nargin < 4 || isempty(objectvarname)
    objectvarname = 'xmean';
  end

  % load data
  d.x = load([filenameprefix objectvarname filenameextension]); 
  % d.x = load([filenameprefix 'xmean' filenameextension]); 
  % d.x = load([filenameprefix 'xrecentbest' filenameextension]); 
  d.f = load([filenameprefix 'fit' filenameextension]); 
  d.std = load([filenameprefix 'stddev' filenameextension]);
  d.D = load([filenameprefix 'axlen' filenameextension]);

  % interpret entries in figNb for cutting out some data
  if length(figNb) > 1
    iend = inf;
    istart = figNb(2);
    if length(figNb) > 2
      iend = figNb(3);
    end
    figNb = figNb(1);
    d.x = d.x(d.x(:,1) >= istart & d.x(:,1) <= iend, :);
    d.f = d.f(d.f(:,1) >= istart & d.f(:,1) <= iend, :);
    d.std = d.std(d.std(:,1) >= istart & d.std(:,1) <= iend, :);
    d.D = d.D(d.D(:,1) >= istart & d.D(:,1) <= iend, :);
  end

  % decide for x-axis
  iabscissa = 2; % 1== versus iterations, 2==versus fevals
  if mod(figNb,100) == 1
    iabscissa = 1; % a short hack
  end
  if iabscissa == 1
    xlab ='iterations'; 
  elseif iabscissa == 2
    xlab = 'function evaluations'; 
  end

  figure(figNb); 
  if size(d.x, 2) < 100
    minxend = 1.03*d.x(end, iabscissa);
  else
    minxend = 0;
  end

  foffset = 1e-99;
  dfit = d.f(:,6)-min(d.f(:,6)); 
  dfit(dfit<1e-98) = NaN;
  subplot(2,2,1); hold off; 
  % additional fitness data, for example constraints values
  if size(d.f,2) > 8
    dd = abs(d.f(:,9:end)) + 10*foffset;
    dd(d.f(:,9:end)==0) = NaN; 
    subplot(2,2,1);semilogy(d.f(:,iabscissa), dd, '-m'); hold on; 
    if size(d.f,2) > 12
      subplot(2,2,1);semilogy(d.f(:,iabscissa),abs(d.f(:,[7 8 11 13]))+foffset,'-k'); hold on;
    end
  end

  idx = find(d.f(:,6)>1e-98);  % positive values
  if ~isempty(idx)  % otherwise non-log plot gets hold
    subplot(2,2,1);semilogy(d.f(idx,iabscissa), d.f(idx,6)+foffset, '.b'); hold on; 
  end
  idx = find(d.f(:,6) < -1e-98);  % negative values
  subplot(2,2,1);semilogy(d.f(idx, iabscissa), abs(d.f(idx,6))+foffset,'.r'); hold on; 
  subplot(2,2,1);semilogy(d.f(:,iabscissa),abs(d.f(:,6))+foffset,'-b'); hold on;
  semilogy(d.f(:,iabscissa),dfit,'-c'); hold on;
  subplot(2,2,1);semilogy(d.f(:,iabscissa),(d.f(:,4)),'-r'); hold on; % AR
  subplot(2,2,1);semilogy(d.std(:,iabscissa),(d.std(:,3)),'-g'); % sigma
  ax = axis;
  ax(2) = max(minxend, ax(2)); 
  axis(ax);
  
  yannote = 10^(log10(ax(3)) + 0.05*(log10(ax(4))-log10(ax(3)))); 
  text(ax(1), yannote, ...
       [ 'f=' num2str(d.f(end,6), '%.15g') ]);

  title('abs(f) (blue), f-min(f) (cyan), Sigma (green), Axis Ratio (red)');
  grid on; 

  subplot(2,2,2); hold off; plot(d.x(:,iabscissa), d.x(:,6:end),'-'); 
  ax = axis;
  ax(2) = max(minxend, ax(2)); 
  axis(ax);
  if size(d.x, 2) < 100
    yy = linspace(ax(3), ax(4), size(d.x,2)-5)';
    [yyl idx] = sort(d.x(end,6:end));
    [muell idx2] = sort(idx);
    hold on;
    plot([d.x(end,iabscissa) ax(2)]', [d.x(end,6:end)' yy(idx2)]', '-');
    plot(repmat(d.x(end,iabscissa),2), [ax(3) ax(4)], 'k-');
    %plot(repmat(o.x(end),2), [yyl(end) ax(4)], 'k-');
    %plot(repmat(o.x(end),2), [ax(3) yyl(1)], 'k-');
    for i = 1:length(idx)
      text(ax(2), yy(i), ['x(' num2str(idx(i)) ')=' num2str(d.x(end,5+idx(i)))]);
    end
  end
  title(['Object Variables (' num2str(size(d.x, 2)-5) '-D)']);grid on;

  subplot(2,2,3); hold off; semilogy(d.D(:,iabscissa), d.D(:,6:end), '-');
  ax = axis;
  ax(2) = max(minxend, ax(2)); 
  axis(ax);
  title('Principle Axes Lengths');grid on;
  xlabel(xlab); 

  subplot(2,2,4); hold off; 
  % remove sigma from stds
  d.std(:,6:end) = d.std(:,6:end) ./ (d.std(:,3) * ones(1,size(d.std,2)-5));
  semilogy(d.std(:,iabscissa), d.std(:,6:end), '-'); hold on; 
  semilogy(d.std(:,iabscissa), [d.std(:,3).*max(d.std(:,6:end)')' ...
                      d.std(:,3).*min(d.std(:,6:end)')'], '-m', 'linewidth', 2);
  ax = axis;
  ax(2) = max(minxend, ax(2)); 
  axis(ax);
  if size(d.std, 2) < 100
    yy = logspace(log10(ax(3)), log10(ax(4)), size(d.std,2)-5)';
    [yyl idx] = sort(d.std(end,6:end));
    [muell idx2] = sort(idx);
    hold on;
    plot([d.std(end,iabscissa) ax(2)]', [d.std(end,6:end)' yy(idx2)]', '-');
    plot(repmat(d.std(end,iabscissa),2), [ax(3) ax(4)], 'k-');
    %plot(repmat(o.x(end),2), [yyl(end) ax(4)], 'k-');
    %plot(repmat(o.x(end),2), [ax(3) yyl(1)], 'k-');
    for i = 1:length(idx)
      text(ax(2), yy(i), [' ' num2str(idx(i))]);
    end
    text(d.std(end,iabscissa), d.std(end,3)*max(d.std(end,6:end)), 'max');
    text(d.std(end,iabscissa), d.std(end,3)*min(d.std(end,6:end)), 'min');
  end
  title('Standard Deviations in Coordinates');grid on;
  xlabel(xlab);

  if figNb ~= 324
    zoom on; 
  end
  drawnow;


