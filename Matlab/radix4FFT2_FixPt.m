function [R, idxSet] = radix4FFT2_FixPt(s)
    % This is a radix-4 FFT, using decimation in frequency
    % The input signal can be floating point or fixed point
    % Works with real or complex input
 
    % Initialize variables and signals
    % NOTE: The length of the input signal should be a power of 4: 4, 16, 64, 256, etc.
    N = length(s);
    M = log2(N)/2;
 
    % Initialize variables for floating or fixed point sim
    if isfi(s)
        NT = numerictype(s);     % scalar
        FM = fimath(s);          % scalar
        wl = NT.WordLength;      % scalar
        W=fi(exp(-1*2j*pi*(0:N-1)/N), 1, wl, wl - 2, FM);           % 1x256 vector
        S = fi(complex(zeros(size(s))), NT, FM);                    % 1x256 vector
        R = fi(complex(zeros(size(s))), 1, wl, wl  - 1 - 2*M, FM);  % 1x256 vector
        sTemp = fi(complex(zeros(size(s))), NT, FM);                % 1x256 vector
    else
        W=exp(-1*2j*pi*(0:N-1)/N);
        S = complex(zeros(size(s)));
        R = complex(zeros(size(s)));
        sTemp = complex(zeros(size(s)));
    end
    
    % Write input index in file
    n=repmat(0:63,4,1);
    n=n(:);
    m=repmat(64*(0:3),1,64).';
    idx= n+m;
      
%     fid = fopen('input_index_sequence.txt','w');
%     fprintf(fid,'int input_idx[256] = { ');
%     for i=1 : length(idx)
%         if i==length(idx)
%             str =num2str(idx(i),'%d');
%         else
%             str =[num2str(idx(i),'%d'),','];
%         end
%  
%         fprintf(fid,str);
%     end
%     fprintf(fid,'}; \r\n'); 
%     fclose(fid);
    
    % Write Twiddle Factor in file
%     TW_float = exp(-1*2j*pi*(0:N-1)/N);
%     fid = fopen('TwiddleFactor_256point.txt','w');
%     ct = 1;
%     fprintf(fid,'const cfix_W14_F12 TF_TABLE[256]; \n');
%     for i=1 : 256
%         
%         str =['TwiddleFactor[',num2str(i-1),'].real()=',num2str(real(TW_float(i)),'%f'),';'];
%         ct=ct+1;
%         if ct == 7, fprintf(fid,'\r\n'); ct = 1; end        
%         fprintf(fid,str);
%     end
%     
%     fprintf(fid,'\r\n\n\n')
%     
%     ct = 1;
%     for i=1 : 256
%         
%         str =['TwiddleFactor[',num2str(i-1),'].imag()=',num2str(imag(TW_float(i)),'%f'),';'];
%         ct=ct+1;
%         if ct == 7, fprintf(fid,'\r\n'); ct = 1; end        
%         fprintf(fid,str);
%     end    
%     
%     fclose(fid);
% 
%     fid = fopen('radix4_indexTable.txt','w');
% 
%     fprintf(fid,'unsigned char IdxTable[NSTAGE][NFFT/4]={ \n')
%     
%     for stage = 0 : 3
%         n= 1 : 64;
%         idx = (floor((n-1)/(4^stage)) *(4^stage));
%         fprintf(fid,'{');
%         for gg = 1 : length(idx)
%             if gg == length(idx) 
%                 str= [num2str(idx(gg)),'}'];
%             else
%                 str= [num2str(idx(gg)),','];
%             end                
%             fprintf(fid,str);                 
%         end
%         
%         if stage == 3
%             fprintf(fid,'\n }; \n');                 
%         else
%             fprintf(fid,',\n');                 
%         end
%         idxTable(stage+1,:)=idx;
%     end    
%         
%     fclose(fid);

    stageMat = zeros(256,4);
    % FFT algorithm
    % Calculate butterflies for first M-1 stages
    sTemp = s;
    idxSet_prev = 1 : N; idxSet=zeros(N,M);
    for stage = 0:M-2
        for n=1:N/4
            % 1st arg : 1x4 vector
            % 2nd arg : scalar
            % 3rd arg : scalar
            % 4th arg : 1x256 vector
            S((1:4)+(n-1)*4) = radix4bfly(sTemp(n:N/4:end), floor((n-1)/(4^stage)) * (4^stage), 1, W);
            idxSet((1:4)+(n-1)*4,stage+1) = idxSet_prev(n:N/4:N);
        end
        idxSet_prev = idxSet(:,stage+1);
        sTemp = S;
        debugS = S.double.';
        stageMat(:,stage+1) = S;
    end
         

    % Calculate butterflies for last stage
    for n=1:N/4
        S((1:4)+(n-1)*4) = radix4bfly(sTemp(n:N/4:end), floor((n-1)/(4^stage)) * (4^stage), 0, W);
        idxSet((1:4)+(n-1)*4,M) = idxSet_prev(n:N/4:N);
        stageMat(:,4) = S;
        debugS = S.double.';
    end
                   
    % Rescale the final output
    R = S*N;

end


function Z = radix4bfly(x,segment,stageFlag,W)
    % Radix-4 Algorithm
    % For the last stage of a radix-4 FFT all the ABCD multiplers are 1.
    % Use the stageFlag variable to indicate the last stage
    % stageFlag = 0 indicates last FFT stage, set to 1 otherwise

    %% Initialize variables and scale by 1/4
    a=bitsra(x(1),2); %x(1)*.25;
    b=bitsra(x(2),2); %x(2)*.25;
    c=bitsra(x(3),2); %x(3)*.25;
    d=bitsra(x(4),2); %x(4)*.25;
    
    %% A=a+b+c+d;    
    tmpA = a.real+b.real+c.real+d.real ...
             + 1i*(a.imag+b.imag+c.imag+d.imag);                 
    % Force tmpA to be casted to A    
    A     = fi(tmpA, 1, 14, 13);          
    
    %% B=(a-b+c-d)*W(2*segment*stageFlag + 1);
    tmpH_B = a.real-b.real+c.real-d.real ...
             + 1i*(a.imag-b.imag+c.imag-d.imag);                  
    % Force tmpH_B to be casted to H_B    
    H_B        = fi(tmpH_B, 1, 14, 12); % type casting     
    
    TF_f       = W(2*segment*stageFlag + 1);    
    TF_fi      = fi(TF_f, 1, 14, 12);    % type casting    
    B_f_full   = (H_B.real*TF_fi.real - H_B.imag*TF_fi.imag) ...
                + 1i*(H_B.real*TF_fi.imag + H_B.imag*TF_fi.real);    
    B          = fi(B_f_full,1,14,13);
    
    %% C=(a-b*1i-c+d*1i)*W(segment*stageFlag + 1);    
    %  b*1i = -b.imag + 1i*b.real --> -b*1i = b.imag - 1i*b.real
    %  d*1i = -d.imag + 1i*d.real
    
    tmpH_C = a.real+b.imag-c.real-d.imag ...
             +  1i*(a.imag-b.real-c.imag+d.real);            
         
    % Force tmpH_C to be casted to H_C    
    H_C     = fi(tmpH_C.double, 1, 14, 12);      
    TF_f    = W(segment*stageFlag + 1);
    TF_fi   = fi(TF_f, 1, 14, 12);
    
    C_fi_full  = (H_C.real*TF_fi.real - H_C.imag*TF_fi.imag) ...
                + 1i*(H_C.real*TF_fi.imag + H_C.imag*TF_fi.real);
    C          = fi(C_fi_full,1,14,13);

    %% D=(a+b*1i-c-d*1i)*W(3*segment*stageFlag + 1);
    %  b*1i = -b.imag + 1i*b.real 
    %  d*1i = -d.imag + 1i*d.real --> -d*1i = d.imag - 1i*d.real   
    
    tmpH_D = a.real-b.imag-c.real+d.imag...
             + 1i*(a.imag+b.real-c.imag-d.real);           
    % Force tmpH_C to be casted to H_C    
    H_D        = fi(tmpH_D.double, 1, 14, 12);      
    TF_f       = W(3*segment*stageFlag + 1);    
    TF_fi      = fi(TF_f, 1, 14, 12);
    D_fi_full  = (H_D.real*TF_fi.real - H_D.imag*TF_fi.imag) ...
                 + 1i*(H_D.real*TF_fi.imag + H_D.imag*TF_fi.real); 
    D          = fi(D_fi_full,1,14,13);  

    % output
    Z = [A B C D];

end
