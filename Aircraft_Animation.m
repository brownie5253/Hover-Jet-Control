function Aircraft_Animation(time, qx, qy, qa, qxr, qyr, qar)

%  Load the variable from simulation
%  time is the simulation time
%  qx is the x position of the plane
%  qy is the y position of the plane
%  qa is the pitch angle of the plane
%  qxr is the equilibrium x position of the plane
%  qyr is the equilibrium y position of the plane
%  qar is the equilibrium pitch angle of the plane

%%
clc;
l=0.2;
disp('Animation started');

%% Auxiliar variables
                                          
hc=0.04;                                            
xp1plane=qx(1)+(qx(1)*0.3);                                       
xp2plane=-(qx(1)+(qx(1)*0.3));           
yp1plane=-1.5*hc; 
lw=xp1plane/10; % wing legth


%% Allocation space movie

mov(1:length(time)) = struct('cdata',[],'colormap',[]);

%% Dimmension of the figure window for the movie

scrsz = get(0,'ScreenSize');
figmovie=figure('Name','Movie: Aircraft Control','Position',[0 0 scrsz(3)*2.5/3 scrsz(3)*1.5/2.9]);

%% For each frame plot aircraft

for k=1:length(time)

%% Set the labels for each frame of the animation

figmovie;clf
axes('NextPlot','replacechildren','tag','plot_axes')
title('Hovering Aircraft','FontSize',18)
xlabel('x [m]','FontSize',18)
ylabel('y [m]','FontSize',18)
text(xp1plane/2+xp2plane/2,-1.4*l,sprintf('Time %0.1f sec',time(k)),'FontSize',18)
hold on;

%% Draw the suporting plane base for the cart

xplane = [xp2plane xp1plane];
yplane = [yp1plane yp1plane];
area(xplane,yplane,'basevalue',-hc,'facecolor',[0.5 .5 0.5]);


%% Initial position of the cart and pendulum

plot(qx(1),qy(1),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','w','MarkerSize',60);
plot([qx(1) + lw*cos(qa(1)); qx(1) - lw*cos(qa(1))], [qy(1) + lw*sin(qa(1)); qy(1) - lw*sin(qa(1))],'k--');


%% Reference position of the cart and pendulum
   
plot(qxr,qyr,'Marker','o','MarkerEdgeColor',[0 .7 0],'MarkerFaceColor','w','MarkerSize',60);
plot([qxr + lw*cos(qar);qxr - lw*cos(qar)], [qyr + lw*sin(qar);qyr - lw*sin(qar)],'k--');


%% Current position of the cart and pendulum

plot([qx(k) + lw*cos(qa(k));qx(k) - lw*cos(qa(k))], [qy(k) + lw*sin(qa(k));qy(k) - lw*sin(qa(k))],'k--');
plot(qx(k),qy(k),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','[0 0 0]','MarkerSize',60);

%% x-axis and y-axis limits

xlim([-xp1plane xp1plane])
ylim([-(qy(1)+(qy(1)*0.25)) (qy(1)+(qy(1)*0.25))])
grid on
hold off


%% Record frame data
mov(k) = getframe(gcf);

end

%% Create AVI file.

vidObj = VideoWriter('Aircraft_Animation.avi');      % Create a video object
vidObj.FrameRate = 10;                                    % Set frames per second in video object
open(vidObj);                                             % Open video object
writeVideo(vidObj,mov);                                   % Write the frames mov in video object
close(vidObj)                                             % Close video object


disp('Animation finished - Saved in Aircraft_Animation.avi')