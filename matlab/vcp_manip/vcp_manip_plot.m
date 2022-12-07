function [] = vcp_manip_plot(fnum, type, data, col, x, u, t)
%VCP_MANIP_PLOT  Plot the trajectory according to the passed type.
%   [] = VCP_MANIP_PLOT(fnum, type, data, col, x, u, t) show in
%           figure(fnum). Implemented plot types are:
%               - "end" x(t), y(t) [2,1]
%               - "theta" theta(t) [n,1]
%               - "input" Plot theta_dot(t) [n,1]
%
%   Copyright (c) 2022, University of Colorado Boulder


switch type
  case "end"
    tiles = [1,4];
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(2,3);
      labels = ["x(t)", "y(t)"];
      for i = 1:2
        nexttile(tiles(i),[1,1]);
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$",labels(i),"$ [m]"),"Interpreter","Latex");
        grid on
        hold on
      end
      nexttile(2,[2,2])
      xlabel("$x$ [m]","Interpreter","Latex");
      ylabel("$y$ [m]","Interpreter","Latex");
      grid on
      hold on
    end
    figure(fnum);
    for i = 1:2
      nexttile(tiles(i));
      hold on
      plot(t,x(i,:),col,'LineWidth',1.5);
    end
    nexttile(2)
    hold on
    plot(x(1,:),x(2,:),'--k');
    % Plot a sparse pose history
    for i = ceil(linspace(1,size(t,2),min(5,size(t,2))))
      if i == 1
        plot_pose(x(:,i), 'r', data)
      else
        plot_pose(x(:,i), col, data)
      end
    end
    scatter(0,0,200,'^k','filled')
    
  case "theta"
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(data.n,1);
      for i = 1:data.n
        nexttile(i);
        plot([t(1),t(end)],rad2deg([data.th_ub(i), data.th_ub(i)]),'--r')
        hold on
        plot([t(1),t(end)],rad2deg([data.th_lb(i), data.th_lb(i)]),'--r')
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$\theta_{",num2str(i),"}(t)$ [deg/s]"),"Interpreter","Latex");
        grid on
      end
    end
    figure(fnum);
    for i = 1:data.n
      nexttile(i);
      hold on
      plot(t,rad2deg(x(2+i,:)),col,'LineWidth',1.5);
    end


  case "input"
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(data.n,1);
      for i = 1:data.n
        nexttile(i);
        plot([t(1),t(end)],rad2deg([data.u_lim, data.u_lim]),'--r')
        hold on
        plot([t(1),t(end)],-rad2deg([data.u_lim, data.u_lim]),'--r')
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$\dot{\theta}_{",num2str(i),"}(t)$ [deg/s]"),"Interpreter","Latex");
        grid on
      end
    end
    figure(fnum);
    for i = 1:data.n
      nexttile(i);
      hold on
      plot(t,rad2deg(u(i,:)),col,'LineWidth',1.5);
    end
 
  otherwise
    error(strcat("ERROR: Type ", type, " is not implemented"))
end
end

function [] = plot_pose(x, col, data)
  r_prev = [0;0];
  for j = 1:data.n
    r = vcp_manip_joint(x(3:end), j, data);
    plot([r_prev(1), r(1)], [r_prev(2), r(2)], 'Color', [0.9290 0.6940 0.1250],'LineWidth',4);
    r_prev = r;
  end
  for j = 1:data.n-1
    r = vcp_manip_joint(x(3:end), j, data);
    scatter(r(1),r(2),100,'k','filled')
  end
  scatter(x(1),x(2),100,"hexagram",col,'Filled')
end