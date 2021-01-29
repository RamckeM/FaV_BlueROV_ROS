close all;
clear all;
A = readmatrix('data.csv','Delimiter','],[ ');
A(:,1) = 1;
A(:,end) = 1;
[m,n] = size(A);
disp(num2str(m))
%%
delta = 2/40;
map = A(end,:);
map = reshape(map,[40,80]);
X = (delta/2):delta:(2-delta/2);
Y = (delta/2):delta:(4-delta/2);
% 
% surf(Y,X,map,'EdgeColor','none')
% view(0,90)
% delta = delta*2
movie = 1;
if movie == 1
    vidObj = VideoWriter(strcat('Video','.avi'));
    vidObj.FrameRate=1.5;
    vidObj.Quality = 100;
    open(vidObj);
    figure('units','pixels','position',[0 0 1920 1080])
for i=1:m
    disp(num2str(i))
    map = A(i,:);
    map = reshape(map,[40,80]);
    map(:,1) = 0.5;
    map(1,:) = 0.5;
    map(end,:) = 0.5;
    map(:,end) = 0.5;
    max(max(map))
    find(map>=0.9)
    surf(Y,X,map)%,'EdgeColor','none')
%     hold on
% %     imagesc([delta/2, 4-delta/2],[delta/2, 2-delta/2],map)
%     pcolor(Y,X,map)
%     hold off
    view(0,90)
    title(strcat('time=',num2str(i)))
    xlabel('x')
    ylabel('z')
    colorbar
    caxis([0,1])
    shading interp; % Setting shading
%     grid on
%     colormap('parula'); % Setting colormap
    lightangle(-35,50)

%     colormap winter;% 
%     view(0,45)
    drawnow
    F(i) = getframe(gcf); % capture the complete figure which includes the colorbar;
    writeVideo(vidObj,F(i));
end
close(vidObj);
end
