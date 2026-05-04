% Adaptive Driving Beam (ADB) Implementation

function illumination = ADB(x, y)
    % the illumination data is a 5 element array that holds the shadow range
    % for both headlights and the brightness value for that range
    
    % if the car is behind you, or outside the range of the headlights,
    % return Not a Number as result
    theta = atand(x/y);
    if  y <= 0 || theta < -15 || theta > 15
        illumination = [NaN, NaN, NaN, NaN, NaN];
        return;
    end

    % assuming the camera is at the center of the car, the distance between
    % the lights and the camera is approximately (average width of a car)/2
    delta_y = 0.9;
    safety_margin = 0.5; % in degrees
    
    % initial theta values, the illumination control module would 
    % normally get these values directly from the camera
    thetaL = atand((x-delta_y) / y);
    thetaR = atand((x+delta_y) / y);
    
    % adjustment due to the parallax effect (camera and lights are at an offset)
    parallax_error = atand(delta_y/ y);

    % in the left lamp we always *add* the error to bring the range towards the right
    left_lamp = [thetaL + parallax_error - safety_margin, thetaR + parallax_error + safety_margin];
    
    % in the right lamp we always *subtract* the error to bring the range towards the left
    right_lamp = [thetaL - parallax_error - safety_margin, thetaR - parallax_error + safety_margin];
    
    illumination = [left_lamp, right_lamp, L(y)];
end

function intensity = L(d)
    % the intensity funcion L determines how bright do you want the lights
    % to be in the shadow range of a detected target, based on the distance

    % these are based on the SAE-compliant distance thresholds
    dmax = 120;    % distance in meters where dimming starts
    dmin = 30;     % distance where light intensity reaches zero
    
    % in the intermediate range the intensity will smoothly go from lambda to 0,
    % which means that as soon as the car enters the range we instantly decrease
    % 1-lambda of intensity
    lambda = 0.8;
    
    if d > dmax
        intensity = 1; % target is far away, full on brightness
        
    elseif dmin <= d && d <= dmax
        % target is in the transition zone, smoothly dim the light
        cosine_term = cos(pi * (d - dmin) / (dmax - dmin));
        intensity = lambda * (1-cosine_term)/2;
        
    elseif d < dmin
        intensity = 0; % target is too close, zero brightness
    end
end
