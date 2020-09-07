clc;
clear;
%%
names = ['E:\Semester\SS20\IVC_Lab\sequences\foreman20_40_RGB\foreman0020.bmp'];
cellnames = cellstr(names);
scales = 0.12:0.4:6.5;

for scaleIDX = 1 : numel(scales)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         still image codec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    qScale = scales(scaleIDX);
    original_image = double(imread(strjoin(cellnames)));
    first_frame = double(imread(strjoin(cellnames)));
    [first_frame_y, first_frame_cb, first_frame_cr] = ictRGB2YCbCr(first_frame);
    [recon_Y, recon_Cb, recon_Cr, bit_rate_curr] = Training(first_frame_y, first_frame_cb, first_frame_cr, qScale);
    recon_image = cat(3, recon_Y, recon_Cb, recon_Cr);
    [recon_r, recon_g, recon_b] = ictYCbCr2RGB(recon_image);
    recon_image_RGB = cat(3, recon_r, recon_g, recon_b);
    MSE = calcMSE(original_image, recon_image_RGB);
    PSNR_still(scaleIDX) = calcPSNR(8, MSE);
    bit_rate_still(scaleIDX) = bit_rate_curr;

    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             video codec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ref_image = cat(3, recon_Y, recon_Cb, recon_Cr);
    minimum = -1000;
    maximum = 1000;
    for it = 1:20
        filename = ['E:\Semester\SS20\IVC_Lab\sequences\foreman20_40_RGB\foreman00',int2str(it+20),'.bmp'];
        original_image = double(imread(filename));
        [original_image_Y, original_image_Cb, original_image_Cr] = ictRGB2YCbCr(original_image);
        current_frame = cat(3, original_image_Y, original_image_Cb, original_image_Cr);
        
        % Motion Estimation
        [err_image, MV_est] = FractionalPixelMotionEstimation(ref_image, current_frame, 0.5, 0.5);
        [err_image_R, err_image_G, err_image_B] = ictYCbCr2RGB(err_image);
        err_image = cat(3, err_image_R, err_image_G, err_image_B);
        row_size = size(err_image,1);
        col_size = size(err_image,2);
        [enc_image] = IntraEncode(err_image, qScale, 2);
        if(it == 1)
            occur_error = histc(enc_image, minimum:maximum);
            PMF_error = occur_error/sum(occur_error);
            PMF_MV = stats_margYCbCr(MV_est, -4, 4);
        end
        [BinaryTree_MV, HuffCode_MV, BinCode_MV, Codelengths_MV] = buildHuffman(PMF_MV); 
        [BinaryTree_err, HuffCode_err, BinCode_err, Codelengths_err] = buildHuffman(PMF_error);
        enc_image = enc_image - minimum + 1;
        
        % Motion Code Estimation
        data_MV = ((floor(MV_est(:)) - (-4)) + 1);
        % Encode errors and motion vectors
        bytestream_curr = enc_huffman_new(enc_image, BinCode_err, Codelengths_err);
        bytestream_MV = enc_huffman_new(data_MV, BinCode_MV, Codelengths_MV);
        
        % Decoding Process
        output_data = dec_huffman_new(bytestream_curr, BinaryTree_err, max(size(enc_image)));
        output_data = output_data + minimum - 1; 
        % Intradecode for deocded data
        output_data_err = IntraDecodenew(output_data, row_size, col_size, qScale, 2);
        dec_err = output_data_err; 
        
        % Motion Compensation
        [recon_image, MV_comp] = MotionCompensation_pel(ref_image, MV_est, dec_err, 0.5, 0.5);
        ref_image = recon_image;
        [recon_image_R, recon_image_G, recon_image_B] = ictYCbCr2RGB(recon_image);
        recon_image_RGB = cat(3, recon_image_R, recon_image_G, recon_image_B);
        
        MSE = calcMSE(original_image, recon_image_RGB);
        PSNR(it) = calcPSNR(8, MSE);
        bit_rate(it) = 8*(length(bytestream_curr)+length(bytestream_MV))/(size(original_image,1) * size(original_image,2));
        
    end
    
    PSNR_av(scaleIDX) = mean(PSNR);
    bit_rate_av(scaleIDX) = mean(bit_rate);
        
        
end

%%  Plot
% save Data_1 PSNR_still bit_rate_still PSNR_av bit_rate_av
% save Data_still PSNR_still bit_rate_still
% plot(bit_rate_still, PSNR_still, '-r*', bit_rate_av, PSNR_av, '-b*', bitperpixel_still_no_opt,PSNR_still_no_opt,'--*r',bitperpixel_av_no_opt,PSNR_av_no_opt,'--*b');
% legend('My Optimization1','My Optimization2','baseline1 (chapter4)','baseline2 (chapter5)','Location','southeast');
plot(bit_rate_still, PSNR_still, '-r*', bit_rate_av, PSNR_av, '-b*');
title('RD performance of Optimization, Foreman Sequence')
axis([0.1 4 24 46]);
xlabel('bitrate [bit/pixel]');
ylabel('PSNR [dB]'); 
