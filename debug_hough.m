% This file is used while runtime debugging the MHT.
% Set a breakpoint after calculating the Hough Accumulator Matrix
% and run the script from the command line (to use the global
% workspace variables)

figure('Position', [200, 200, 1200, 500])
subplot(1, 2, 1)
imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,'InitialMagnification','fit');
title('Hough transform of the plane binary image');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);

temp_peaks = houghpeaks(H, num_of_peaks, 'Threshold', ceil(houghpeaks_thres_coeff*max(H(:))));

% optionally mark peaks on the Hough transform display
plot(theta(temp_peaks(:,2)), rho(temp_peaks(:,1)), 'gs');

temp_lines = houghlines(BW, theta, rho, temp_peaks, 'FillGap', 20, 'MinLength', 1);

subplot(1, 2, 2); hold on;
for temp_ctr = 1:length(temp_lines)
    temp_xy = [temp_lines(temp_ctr).point1; temp_lines(temp_ctr).point2];
    plot(temp_xy(:,1), temp_xy(:,2), 'g-', 'LineWidth', 2, 'Color', rand(1,3)); % line
    plot(temp_xy(:,1), temp_xy(:,2), 'ro', 'MarkerSize', 4, 'LineWidth',1); % endpoints
end
hold off;

% you can clean up afterwards with clear temp_*