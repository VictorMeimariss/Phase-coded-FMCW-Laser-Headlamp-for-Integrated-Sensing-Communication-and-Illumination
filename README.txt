!!!-THIS IS THE INITIALLY PLANNED ARCHITECTURE OF THE CODE AND WILL BE CHANGED-!!!

-------------------------------READ BEFORE USING-------------------------------------------
As the code is to reproduce the results of the said paper. To do this, the ego vehicle
is placed at the origin (0,0) and the targets are moving relative to that point.

1) The inputs are plugged into main.m containing the number of vehicles, their true
relative velocity, range from the ego vehicle, camera angle θ, SNR [1]*,
digital phase code [2]* and finally a number of interference echos which did not
originate from the ego vehicle.

-------------------------------------------Loop---------------------------------------------
	3) Using the inputs the "generate_signal" function, generates a signal
	array, which is the result of the noisy echos mixed with the original signal
	and the filtered.

	4) The signal is then processed by the "decouple_signal" function which
	using 2D-FFT a heatmap is generated and used by the CA-CFAR [3]* algorithm
	extracting R, (θ is known from the camera)and t is the specific frame time(R, θ, t) creating
	x,y,t which then is used to return the projected plane matrices: xy, xt and yt.

	5) Using the plane matrices with the "hough_2D" function, the Hough transform "votes"
	between the three plane same point IDs contributing to "winning peaks" and if
	they are the same (AND Logic) the point is chosen as a real target and stored in
	segments which are later plotted by the "main.m" script which also provides
	the RMSE(Root Mean Square Error).

	6) Finally the function "cost_check" compares the new segment to the last one and if its 
	consistent to it, stiches it together creating the finalized solution, if not, a new
	stich is beginning for another car.

	7) To adjust the headlamps based on the tracking "adjust_headlamps" is used by calculating
	the shading interval and using the estimated radar range and camera angle along with equation
	5 from the said paper.

*[1] SNR is an input to simulate over different conditions, for example 20dB for clean
signal, 0 for equal signal and noise loudness and -15dB for bad signal.

*[2] The digital phase code allows distinguishment of incoming signals allowing the ego
vehicle to know which signals came from it and which came from other cars.

*[3] The Ca-CFAR will be soft as is in the said paper by increasing the probability of the false
alarm to 10^-2 or 10^-3(needs checking).