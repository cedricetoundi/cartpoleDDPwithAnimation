function MovieFile = PlotResults(sol)

time = sol.time;
state = sol.state;
control = sol.control;
cost = sol.cost;
target = sol.target;
K = sol.gain;
l = sol.l;

K =squeeze(K);

% Plot Results

figure('Position',[300 100 624 564]);


% Plot States: x [m], v [m/sec], theta [deg], thetadot [deg/sec]

subplot(3,2,1)
hold on
plot(time, state(1,:),'linewidth',3)
plot(time, target(1) * ones(size(state(1,:), 2)),'r','linewidth',3)
xlabel('time')
ylabel(' x [m]')
hold off 
grid;
legend('Posi', 'Target')

subplot(3,2,2)
hold on
plot(time, state(2,:),'linewidth',3)
plot(time, target(2) * ones(size(state(2,:), 2)),'r','linewidth',3)
xlabel('time')
ylabel(' v [m/sec]')
hold off
grid;
legend('Velocity', 'Target')

subplot(3,2,3)
hold on
plot(time, state(3,:) * 180/pi,'linewidth',3)
plot(time, target(3) * ones(size(state(3,:), 2)),'r','linewidth',3)
xlabel('time')
ylabel(' theta [deg]')
hold off
grid;
legend('Angular Position', 'Target')

subplot(3,2,4)
hold on
plot(time, state(4,:)* 180/pi,'linewidth',3)
plot(time, target(4) * ones(size(state(4,:), 2)),'r','linewidth',3)
xlabel('time')
ylabel(' thetadot [deg/sec]')
hold off 
grid;
legend('Angular Velocity', 'Target')

% Plot cost vs number of iterations
subplot(3,2,5)
plot(cost ,'linewidth',3)
xlabel('Iterations')
ylabel('Cost')
grid;

% Plot controller gains
figure('Position',[600 100 524 564]);
hold on
plot([1:size(K, 2)], K(1,:))
plot([1:size(K, 2)], K(2,:))
plot([1:size(K, 2)], K(3,:))
plot([1:size(K, 2)], K(4,:))
xlabel('Time')
ylabel('Gains')
title('Evolution of Gains with Time')
legend('Position Gain', 'Velocity Gain', 'Angular Position Gain', 'Angular Velocity Gain')
grid;
hold off

figure

% ...

% Animation

% Create a video writer object
videoFile = VideoWriter('cartpole_stabilization_animation.mp4', 'MPEG-4');
videoFile.FrameRate = 10;  % Adjust the frame rate as needed
open(videoFile);

% Define the threshold angle for the pole to be considered upright (adjust as needed)
uprightThreshold = 5 * pi / 180;  % 5 degrees

% Define the cart's width (adjust as needed)
cartWidth = 0.1;

% Fixed pole length
poleLength = 1;

% Animation loop
for t = 1:length(time)
    cartPosition = state(1, t);  % Extract current cart position
    poleAngle = state(3, t);     % Extract current pole angle

    % Calculate the position of the cart and pole endpoints
    cartX = cartPosition;
    cartY = 0;
    poleX = cartX + sin(poleAngle) * poleLength;  % Adjusted pole position calculation
    poleY = cos(poleAngle) * poleLength;          % Adjusted pole position calculation

    % Plot the cart (as a box) and pole
    cartLeftX = cartX - cartWidth / 2;
    cartRightX = cartX + cartWidth / 2;

    plot([cartLeftX, cartRightX, cartRightX, cartLeftX, cartLeftX], ...
         [0, 0, cartWidth, cartWidth, 0], 'b-', 'LineWidth', 2);  % Box representing the cart
    hold on;

    % Plot the pole
    plot([cartX, poleX], [cartY, poleY], 'ro-', 'MarkerSize', 10, 'LineWidth', 2);

    % Highlight the pole's upright position
    if abs(poleAngle) <= uprightThreshold
        plot(poleX, poleY, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');  % Upright position
    end
    axis equal;  % Ensure equal aspect ratio
    xlabel('Cart Position');
    ylabel('Pole Position');
    title('Cartpole System Animation');

    % Set axis limits (adjust as needed)
    axis([-3, 3, -1, 1]);

    % Pause for animation speed
    pause(0.01);

    % Capture the current frame for the video
    frame = getframe(gcf);
    writeVideo(videoFile, frame);

    hold off;
end

%Close the video file
close(videoFile);

% ...



end



% 
% 
% 
% 
% 
