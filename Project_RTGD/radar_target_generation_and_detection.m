clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Frequency of operation = 77GHz
%Max Range = 200m
%Range Resolution = 1 m
%Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_operation=77e9;
range_max=200;
range_resolution=1;
v_max=100;

%speed of light = 3e8
c=3e8;

%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant

%initial position
R=110;

%initial velocity
v=-20;


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

%Bandwidth
B=c/(2*range_resolution);

%Chirp Time
Tchirp=5.5*2*range_max/c;

%Slope
slope=B/Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i)=R+v*t(i);
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i)+slope*t(i)^2/2));
    td(i) = 2*r_t(i)/c;
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+slope*(t(i)-td(i))^2/2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_reshape=reshape(Mix,[Nr,Nd]);

 % *%TODO* 
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft=fft(Mix_reshape,[],1);
%size(sig_fft)
sig_fft=sig_fft/length(sig_fft);
%size(sig_fft)

 % *%TODO*
% Take the absolute value of FFT output
sig_fft=abs(sig_fft);
%size(sig_fft)

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
sig_fft=sig_fft(1:length(sig_fft)/2+1);
%size(sig_fft)

%plotting the range
figure ('Name','Range from First FFT')
%subplot(2,1,1)

% *%TODO* :
% plot FFT output 
plot(sig_fft)  
axis ([0 200 0 1]);

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
%Tr: training cells rows
%Tc: training cells columns
Tr=2; 
Tc=6;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
%Gr: guard cells rows
%Gc: guard cells columns
Gr=2;
Gc=4;

% the number of training cells should be averaged
N_T=(1+2*Tr+2*Gr)*(1+2*Tc+2*Gc)-(1+2*Gr)*(1+2*Gc);

% *%TODO* :
% offset the threshold by SNR value in dB
offset=10;

%number of rows of matrix RDM
Nr=Nr/2;

%number of columns of matrix RDM
Nc=Nd;

% *%TODO* :
% initialize the matrix for storing the output result;
% in this way, 0s are directly set for the cells will not be thresholded;
output = zeros(Nr,Nc);

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

for i = Tr+Gr+1:(Nr-(Gr+Tr))  
    for j = Tc+Gc+1:(Nc-(Gc+Tc))
        % the value of threshold at each iteration
        threshold = 0;
        % calculate the threshold value following the process
        % the process is on the Udacity course website
        for m = i-(Tr+Gr):i+(Tr+Gr)
            for n = j-(Tc+Gc):j+(Tc+Gc)
                if m >= i-Gr && m <= i+Gr && n >= j-Gc && n <= j+Gc
                    % do nothing here, in the gard region
                else
                    threshold = threshold + db2pow(RDM(m,n));
                end 
            end
        end
        threshold=threshold/N_T;
        threshold=pow2db(threshold)+offset;
        
        %perform the threshold process;
        %store the final result in matrix noise_level;
        if(RDM(i,j)<threshold)
            output(i,j)=0;
        else
            output(i,j)=1;
        end
        
    end 
end

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,output);
colorbar;


 
 