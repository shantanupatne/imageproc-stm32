% This script establishes serial communication with the board,
% capture image frames from webcam and send those frames to the board 
% to process. It then waits for the processed frame to be sent 
% from the board. It plots the original frame, processed frame 
% and their histograms for comparison.

close all; clear all;
% need to change the COM port number based on your setting, you can use
% TeraTerm to check which COM port the USB adapter is connected to
s1 = serialport('COM8', 115200); 
s1.Parity = 'none';
s1.DataBits = 8;
s1.StopBits = 1;
s1.Timeout = 120;

% need to install the MATLAB Support Package for USB Webcams
% https://www.mathworks.com/matlabcentral/fileexchange/45182-matlab-support-package-for-usb-webcams
camlist=webcamlist;
cam=webcam(camlist{1,1});

% grab multiple frames
%num_of_frames = 12;
num_of_frames = 1;
for i=1:1:num_of_frames
	A=snapshot(cam); 
   
	B = rgb2gray(A); % transform the colored image to gray level image and store in B

   	B = imresize(B,[96 128],'bicubic'); % resize the image to 96x128

	% transform the image to a 1D vector so that it may be streamed over the serial port
	C=B(:)'; 
    size(C)	
    write(s1,C,"uint8"); % write the vector to serial port and send
    
	WaitForFrame=1; % wait to receive from board
	% while loop waiting for a frame to be sent from the board
	while (WaitForFrame==1)
	    if (s1.NumBytesAvailable == length(C)) % wait for a whole frame
	        WaitForFrame = 0;
        end
        %disp('waiting...');
	end

	% once the frame is in the serial buffer, read that   
	D = read(s1,s1.NumBytesAvailable,"uint8"); % read received frame in 1D vector format
	D = D';
    D = D * (256 / (max(D) + 1));
	[m,n] = size(B); 
	E = ones(m,n); % define a matrix of ones having the same size as the frame 

	% vector to matrix transformation; E contains the received frame in matrix format
	E(:) = D(:); 

    % plot the orignal image frame and the frame received from side by side
	subplot(2,2,1)
	imshow(B); % show the original image grabbed from the USB webcam
	drawnow;
	title('Input Image');

	subplot(2,2,2)
	imshow(uint8(E)); % show the processed one received from the board
	title('Output Image');
	drawnow;
    
    % plot the original image histogram
    subplot(2,2,3)
    imhist(uint8(B));
    title('Input Histogram');
    drawnow;
    
    % plot the processed image histogram
    subplot(2,2,4)
    imhist(uint8(E));
    title('Output Histogram');
    drawnow;

end	%for i=1:1:num_of_frames

% close the webcam and the serial port
clear('cam');
clear s1; % close port
