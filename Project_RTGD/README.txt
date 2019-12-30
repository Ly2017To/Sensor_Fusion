This folder contains the submission for Project of Sensor Fusion Course at Udacity.

Project: Radar Target Generation and Detection.

Matalab Code: radar_target_generation_and_detection.m

After running the Matlab code, three figures will be generated:

range_FFT.jpg

2D_FFT.jpg

2D_CFAR.jpg

Brief Explanations: 

1.Implementation steps for the 2D CFAR process:

first, based on the output matrix RDM of 2D FFT of the previous step, initialize an output matrix of the same size as RDM with all the values set to 0. Then by following the guide from the website of Udacity step by step, the thereshold value of each CUT can be obtained. Finally, for each CUT, comparing the signal at it and its corresponding threshold value, assign 1 or 0 to output matrix. I have write the comments alongside the code.


2.Selection of Training, Guard cells and offset.

selection of training cells: smaller value of rows (Tr=2), larger value for columns (Tc=6);

Reason: For each CUT, the signal within it own chirp is more important to determine the noise level. Because the signal within each chirp is stored in the column of matrix RDM, more training cells on columns are considered to estimate the noise level.

selection of guard cells: smaller value of rows (Gr=2), larger value for columns (Gc=4); 

Reason: For each CUT, I want to reduce interference and also the training cells should be close to CUT. Because the signal within each chirp is stored in the column of matrix RDM, the closer the cells lie with each other, the more interference will be. 


offset: tried varies values between 5 and 15, and I choose 10 finally.

3.Steps taken to suppress the non-thresholded cells at the edges.

check the first sentence of the explanation of 1. 

