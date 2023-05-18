function [] = vcp_quad_plot(fnum, type, data, col, x, u, t)
%VCP_QUAD_PLOT  Plot the trajectory according to the passed type. 
%   [] = VCP_QUAD_PLOT(fnum, type, data, col, x, u, t) show in
%           figure(fnum). Implemented plot types are:
%               - "3D" Plots the trajectory of x,y,z in 3D
%               - "pos" Plot x(t), y(t), z(t) [3,1]
%               - "eul" Plot phi(t), theta(t), psi(t) [3,1]
%               - "vel" Plot dx(t), dy(t), dz(t) [3,1]
%               - "inp" Plot T(t), p(t), q(t) [[1:2],2]
%
%
%   Copyright (c) 2022, University of Wisconsin-Madison


switch type
  case "3D"
    if(~ishandle(fnum))
      figure(fnum)
      plot3(data.P_wp(1,:),data.P_wp(2,:),data.P_wp(3,:),'or');
      grid on
      xlabel("$x$ [m]","Interpreter","Latex");
      ylabel("$y$ [m]","Interpreter","Latex");
      zlabel("$z$ [m]","Interpreter","Latex");
    end
    figure(fnum)
    hold on
    plot3(x(1,:),x(2,:),x(3,:),col,'LineWidth',1.5);

  case "pos"
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(3,1);
      labels = ["x","y","z"];
      for i = 1:3
        nexttile(i);
        plot([t(1),t(end)],[data.r_ub(i), data.r_ub(i)],'--r')
        hold on
        plot([t(1),t(end)],[data.r_lb(i), data.r_lb(i)],'--r')
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$",labels(i),"(t)$ [m]"),"Interpreter","Latex");
        grid on
      end
    end
    figure(fnum);
    hold on
    for i = 1:3
      nexttile(i);
      plot(t,x(i,:),col,'LineWidth',1.5);
    end
  
  case "eul"
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(2,1);
      labels = ["\phi","\theta"];
      for i = 1:2
        nexttile(i);
        plot([t(1),t(end)],rad2deg([data.epsilon, data.epsilon]),'--r')
        hold on
        plot([t(1),t(end)],rad2deg([-data.epsilon, -data.epsilon]),'--r')
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$",labels(i),"(t)$ [m]"),"Interpreter","Latex");
        grid on
      end
    end
    figure(fnum);
    hold on
    for i = 1:2
      nexttile(i);
      plot(t,rad2deg(x(i+3,:)),col,'LineWidth',1.5);
    end

  case "vel"
    if(~ishandle(fnum))
      figure(fnum)
      tiledlayout(3,1);
      labels = ["\dot{x}","\dot{y}","\dot{z}"];
      for i = 1:3
        nexttile(i);
        hold on
        xlabel("$t$ [s]","Interpreter","Latex");
        ylabel(strcat("$",labels(i),"(t)$ [m]"),"Interpreter","Latex");
        grid on
      end
    end
    figure(fnum);
    hold on
    for i = 1:3
      nexttile(i);
      plot(t,x(i+6,:),col,'LineWidth',1.5);
    end


  case "inp"
    figure(fnum);
    subplot(2, 2, [1 3]); % T
    plot(t,u(1,:),col,'LineWidth',1.5)
    hold on
    if(data.T_ub)
      plot([t(1),t(end)],[data.T_ub,data.T_ub],'--r')
    end
    if(data.T_lb)
      plot([t(1),t(end)],[data.T_lb,data.T_lb],'--r')
    end
    grid on
    xlabel('$t$ [s]','Interpreter','Latex')
    ylabel('$T(t)$ [m/s$^2$]','Interpreter','Latex');
    subplot(2, 2, 2) % p
    plot(t,rad2deg(u(2,:)),col,'LineWidth',1.5)
    hold on
    if(data.omega_max)
      plot([t(1), t(end)],rad2deg([data.omega_max, data.omega_max]),'--r')
      plot([t(1), t(end)],rad2deg([-data.omega_max, -data.omega_max]),'--r')
    end
    grid on
    xlabel('$t$ [s]','Interpreter','Latex')
    ylabel('$p(t)$ [deg/s]','Interpreter','Latex')
    subplot(2, 2, 4) % q
    plot(t,rad2deg(u(3,:)),col,'LineWidth',1.5)
    hold on
    if(data.omega_max)
      plot([t(1), t(end)],rad2deg([data.omega_max, data.omega_max]),'--r')
      plot([t(1), t(end)],rad2deg([-data.omega_max, -data.omega_max]),'--r')
    end
    grid on
    xlabel('$t$ [s]','Interpreter','Latex')
    ylabel('$q(t)$ [deg/s]','Interpreter','Latex')


  case "speed"
    if(~ishandle(fnum))
      figure(fnum)
      if(data.v_max)
        plot([0,data.t_f],[data.v_max,data.v_max],'--r')
      end
      grid on
      xlabel('$t$ [s]','Interpreter','Latex')
      ylabel('$\|\dot{\mathbf{\sigma}}\|_2$ [m/s]','Interpreter','Latex')
    end
    v = vecnorm(x(7:9,:));
    figure(fnum)
    hold on
    figure(fnum);
    plot(t,v,col,'LineWidth',1.5);

  otherwise
    error(strcat("ERROR: Type ", type, " is not implemented"))
end