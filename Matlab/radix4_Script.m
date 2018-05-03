% Radix-4 FFT Test Script
% This file runs three versions of a Radix-4 FFT written in MATLAB:
%
%   radix4FFT1_Float.m computes a radix-4 FFT for floating point data types
%
%   radix4FFT2_FixPt.m computes a fixed point radix-4 FFT (requires Fixed
%   Point Toolbox)
%
% For a description of the radix-4 FFT algorithm see the following link to
% DSPDesignLine.com:
%
%       http://www.dspdesignline.com/showArticle.jhtml;jsessionid=5DBROAJQ3
%       SIWCQSNDLOSKHSCJUNN2JVN?articleID=204400676&queryText=freescale+fft 
%
close all;
% Set up a signal

NFFT = 256;
load fftTestSignal.mat

% ADC normalization
compressedSignal = compressedSignal / (2^10 - 1); 

for ch = 3
    compressedSig = 20*compressedSignal(ch,:);
    
    % Fixed Point Test
    % Run the Radix-4 FFT algorithm with a lower precision data type.    
    % Compare between real signal and fixed-point signal
    nSegment = 10;  
    fixedPointFFToutSet = zeros(1,nSegment); 
    matlabFFToutSet = zeros(1,nSegment);
    
    for segloopCnt = 1 : nSegment
        % Get a part of signal
        segmendIdx = 1 + NFFT*(segloopCnt-1) : NFFT*segloopCnt ;
        compressedSigSegment = compressedSig(segmendIdx);
                
        % Set Fixed Point Parameters
        % The accuracy of the FFT for a shorter word length can be tested by
        % changing the input signal to a 10 bit integer with 9 fractional bits.
        
        wl = 16;
        sfi=fi(compressedSigSegment,1,wl,wl-1);    % Data is Q10,9
        sfi.RoundMode = 'nearest';                 % Fixed Point Rounding, etc.
        sfi.OverflowMode = 'wrap';
        sfi.ProductMode = 'KeepMSB';
        sfi.ProductWordLength = wl*2;
        sfi.SumMode = 'KeepMSB';
        sfi.SumWordLength = wl*2;
    
        % Execute fixed-point FFT function
        [SFI, idxSet] = radix4FFT2_FixPt(sfi);
        
        [SFI_rev,iid] = bitrevorder(SFI);        
        % convert from fixed-point data to floating data
        fixedPointFFTout = SFI_rev.double;
        
        % Calculate FFT using MATLAB function
        floatingPointFFTout = fft(compressedSigSegment);
        
        % Calculate Error
        errs = double(SFI) - floatingPointFFTout;
        Sig = sum(abs(floatingPointFFTout).^2)/NFFT;
        Noise = sum(abs(errs).^2)/NFFT;
        SNR = 10*log10(Sig/Noise);
        sprintf('SNR for fixed vs floating point methods is: %6.2f dB', SNR)
        
        % Contain fft result in a variable
        fixedPointFFToutSet(segmendIdx) = fixedPointFFTout;
        matlabFFToutSet(segmendIdx) = floatingPointFFTout;
    end
          
    % Write result in file
    FileID3 = fopen(['output_ch',num2str(ch),'_real.dat'],'w'); 
    X_COLS=size(fixedPointFFToutSet,2);         
    for c = 1 : X_COLS     
        fprintf(FileID3, '%1.10f  \n',real(fixedPointFFToutSet(c)));            
    end

    fclose(FileID3);

    FileID4 = fopen(['output_ch',num2str(ch),'_imag.dat'],'w'); 
    for c = 1 : X_COLS     
        fprintf(FileID4, '%1.10f  \n',imag(fixedPointFFToutSet(c)));            
    end

    fclose(FileID4);
    
    % display result
    xx = 1 : nSegment * NFFT;
    figure; 
    subplot(2,1,1); plot(xx, real(fixedPointFFToutSet),'--b.',...
        xx,real(matlabFFToutSet),'--ro'); 
    legend( {'Fixed-FFT','Floating-FFT'} );
    title('Real Part');
    subplot(2,1,2); plot(xx, imag(fixedPointFFToutSet),'--b.',...
        xx,imag(matlabFFToutSet),'--ro'); 
    legend( {'Fixed-FFT','Floating-FFT'} );
    title('Imaginary Part');
end

if exist('FFT_HLS','var')
    figure;
    t=0 : length(FFT_HLS)-1 ; %0:1/NFFT:(NFFT-1)/NFFT;
    subplot(211)
    plot(t,real(FFT_HLS),'--b.',t,real(fixedPointFFToutSet),'-.ro');
    xlabel('samples ');ylabel('Amplitude');title('Test Signal - Real Component')
    legend({'HLS','MAT'});
    subplot(212)
    plot(t,imag(FFT_HLS),'--b.',t,imag(fixedPointFFToutSet),'-.ro')
    xlabel('samples ');ylabel('Amplitude');title('Test Signal - Imag Component')
    legend({'HLS','MAT'});
end
