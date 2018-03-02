%% Radix-4 FFT Test Script
% This file runs three versions of a Radix-4 FFT written in MATLAB:
%
%   radix4FFT1_Float.m computes a radix-4 FFT for floating point data types
%
%   radix4FFT2_FixPt.m computes a fixed point radix-4 FFT (requires Fixed
%   Point Toolbox)
%
%   radix4FFT3_FixPtEML.m is an Embedded MATLAB version of the radix-4 FFT
%   that can be used in Simulink.  You can also generate C code for this
%   code (using Real Time Workshop).  This version can also be compiled
%   into a MEX'd executable that runs significantly faster than the fixed
%   point code.
%
% For a description of the radix-4 FFT algorithm see the following link to
% DSPDesignLine.com:
%
%       http://www.dspdesignline.com/showArticle.jhtml;jsessionid=5DBROAJQ3
%       SIWCQSNDLOSKHSCJUNN2JVN?articleID=204400676&queryText=freescale+fft 
%
%% Set up a signal

% clear all; 
% close all;

NFFT = 256;
% load radix4_fft_compressed_sig2.mat
% load adcData_middleband_fft256.mat
% load radix4_fft_compressed_sig_171120.mat
    load adcData_From_ILA_171112.mat
% ad0 = YadcIdeal;
if exist('ad0','var')
    nn = 1;
    compressedSigSet = ad0((256*nn +1):256*(nn+1)).';
    figure; plot(compressedSigSet)
else
        
%     compressedSigSet = YadcIdealNorm.';
% figure; plot(YadcIdealNorm) 
  compressedSigSet = adcDataILA.';
%     figure; plot(adcDataILA) 
end

% ADC normalization
compressedSigSet = compressedSigSet / (2^10 - 1); 

gainf = 20;
a=fi(gainf*compressedSigSet(3,:),1,14,13);
b=a.double.';
xx= 1:length(a);
  figure; plot(xx,a,'--b.',xx,gainf*compressedSigSet(3,:),'--ro') 
% load radix4_fft_test_sig.mat
for ch = 3; %1  : size(compressedSigSet,1)

    compressedSig = gainf*compressedSigSet(ch,:);
    
    %% Fixed Point Test
    % Run the Radix-4 FFT algorithm with a lower precision data type.
    %
    % Compare between real signal and fixed-point signal
    nSegment = 10; %size(compressedSig,2) / NFFT;    
    fixedPointFFToutSet = zeros(1,nSegment); % zeros(1,size(compressedSig,2));
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
    
%         nIdx = 1 : 256;
%         figure; plot(nIdx,compressedSigSegment,'--b.',nIdx,sfi.double,'--ro');
%         xlabel('sampes');ylabel('Amplitude');
%         legend( {'Floating-point signal','Fixed-point signal'} );
 
        % Execute fixed-point FFT function
        [SFI, idxSet] = radix4FFT2_FixPt(sfi);
%         [SFI_rr, idxSet_rr] = radix4_Script_Resource_Reduce(sfi);
        
        [SFI_rev,iid] = bitrevorder(SFI);
        debugS = SFI_rev.double.';
%         [SFI_rr,iid_rr] = bitrevorder(SFI_rr);
%         SFI_HLSTYPE = fi(SFI,1,14,8);
        % convert from fixed-point data to floating data
        fixedPointFFTout = SFI_rev.double;
%         fixedPointFFTout_rr = SFI_rr.double;
%         gi = [1:48, 209:256];
        % Calculate FFT using MATLAB function
        y = fft(compressedSigSegment);

        % Compare between normal ver. and resource reduced ver.
%         xx= 1 : 256;
%         figure; 
%         subplot(2,1,1),plot(xx,real(fixedPointFFTout),'b.--',xx,real(fixedPointFFTout_rr),'ro--',xx(gi),real(fixedPointFFTout_rr(gi)),'kp');
%         title('real');
%         legend( {'Normal','RR ver.'} );
%         xlabel('samples');ylabel('|Magnitude spectrum|');
%         subplot(2,1,2),plot(xx,imag(fixedPointFFTout),'b.--',xx,imag(fixedPointFFTout_rr),'ro--',xx(gi),imag(fixedPointFFTout_rr(gi)),'kp');
%         title('imag');
%         xlabel('samples');ylabel('|Magnitude spectrum|');
%         legend( {'Normal','RR ver.'} );
        
        % Compare results
%         xx= 1 : 256;
%         figure; 
%         subplot(2,1,1),plot(xx,real(y),'--b.',xx,real(fixedPointFFTout),'ro--');
%         title('real');
%         legend( {'MATLAB-FFT','HLS-FFT'} );
%         xlabel('samples');ylabel('|Magnitude spectrum|');
%         subplot(2,1,2),plot(xx,imag(y),'--b.',xx,imag(fixedPointFFTout),'ro--');
%         title('imag');
%         xlabel('samples');ylabel('|Magnitude spectrum|');
%         legend( {'MATLAB-FFT','HLS-FFT'} );

        % Calculate Error
        errs = double(SFI) - y;
        Sig = sum(abs(y).^2)/NFFT;
        Noise = sum(abs(errs).^2)/NFFT;
        SNR = 10*log10(Sig/Noise);
        sprintf('SNR for fixed vs floating point methods is: %6.2f dB', SNR)
        
        % Contain fft result in a variable
        fixedPointFFToutSet(segmendIdx) = fixedPointFFTout;
        matlabFFToutSet(segmendIdx) = y;
        % plotPYYf(double(sfi),NFFT)

        % In this case there is an approximate loss of 12 dB of accuracy compared
        % to the 16 bit FFT computation

        % %% Use emlmex to compile code into executable
        % % The fixed point FFT code can be accelerated by changing the algorithm to
        % % an EML-compliant algorithm and compiling it with the emlmex command. The
        % % emlmex command will produce a mex'd version of the MATLAB algorithm.
        % codegen -o radix4FFT3_MX -args {sfi}  radix4FFT3_FixPtEML
        % 
        % %% Show speed of non-compiled code
        % tic;SFI = radix4FFT3_FixPtEML(sfi);toc
        % tic;SFI = radix4FFT3_FixPtEML(sfi);toc
        % tic;SFI = radix4FFT3_FixPtEML(sfi);toc
        % 
        % %% Show speed of compiled code
        % tic;SFI = radix4FFT3_MX(sfi);toc
        % tic;SFI = radix4FFT3_MX(sfi);toc
        % tic;SFI = radix4FFT3_MX(sfi);toc
        % 
        % % The MEX'd version of the FFT code runs over 600 times faster than the
        % % fixed point MATLAB algorithm.
        % %
        % %% Generate C source code for radix-4 FFT
        % % EML compliant algorithms can be used to generate C source code using Real
        % % Time Workshop. 
        % % rtwcfg = emlcoder.RTWConfig
        % rtwcfg = coder.CodeConfig;
        % 
        % % codegen -v -c rtwcfg -args {sfi} -o radix4FFT_C radix4FFT3_FixPtEML
        % codegen -v -config rtwcfg -o radix4FFT_C -args {sfi} radix4FFT3_FixPtEML
        % % The source code can be inspected in the emcprj directory
        % 
        % %% Use the EML compliant FFT code in Simulink
        % % The attached Simulink model hiperman_4_2007b_FixPt.mdl uses the radix-4
        % % FFT algorithm in the OFDM_RX block (requires Simulink and Communications
        % % Blockset).
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
%     subplot(2,1,1); plot(xx, real(fixedPointFFToutSet),'--b.',xx,real(matlabFFToutSet),'--ro'); %,xx,real(fftHLS),'--kp'); 
%     subplot(2,1,2); plot(xx, imag(fixedPointFFToutSet),'--b.',xx,imag(matlabFFToutSet),'--ro'); %,xx,imag(fftHLS),'--kp'); 
    subplot(2,1,1); plot(xx, real(fixedPointFFToutSet),'--b.',xx,real(matlabFFToutSet),'--ro',xx,real(fftHLS(1:nSegment*256)),'--kp'); 
    legend({'FixedPoint','MATLAB'});
    subplot(2,1,2); plot(xx, imag(fixedPointFFToutSet),'--b.',xx,imag(matlabFFToutSet),'--ro',xx,imag(fftHLS(1:nSegment*256)),'--kp'); 
    legend({'FixedPoint','MATLAB'});
    title(['fixedPoint FFT vs MATLAB','  channel : ',num2str(ch)]);
    
    
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
% load radix4_fft_test_sig.mat
