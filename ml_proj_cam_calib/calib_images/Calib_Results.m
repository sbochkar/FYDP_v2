% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1113.288262609562700 ; 1132.008969340548000 ];

%-- Principal point:
cc = [ 651.237796835080870 ; 360.799043173623830 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.008548221274731 ; -0.300274660657195 ; -0.002112684234929 ; -0.000287874542142 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 13.112681071553695 ; 14.158249846891872 ];

%-- Principal point uncertainty:
cc_error = [ 24.627312669208234 ; 19.942414671832562 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.035098780899151 ; 0.102408171876272 ; 0.004447627446328 ; 0.007498062632063 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 9;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.433959e+00 ; -1.660733e+00 ; 1.047695e+00 ];
Tc_1  = [ 8.667200e+01 ; -5.838929e+01 ; 5.853133e+02 ];
omc_error_1 = [ 1.856384e-02 ; 1.456505e-02 ; 2.046853e-02 ];
Tc_error_1  = [ 1.323828e+01 ; 1.035118e+01 ; 7.001069e+00 ];

%-- Image #2:
omc_2 = [ -1.882058e+00 ; -1.906337e+00 ; 5.702405e-01 ];
Tc_2  = [ 1.331614e+02 ; -8.346572e+01 ; 6.169534e+02 ];
omc_error_2 = [ 1.710640e-02 ; 1.413554e-02 ; 2.316081e-02 ];
Tc_error_2  = [ 1.419674e+01 ; 1.097737e+01 ; 7.777835e+00 ];

%-- Image #3:
omc_3 = [ -1.360206e+00 ; -2.091334e+00 ; 1.429605e+00 ];
Tc_3  = [ 1.056043e+02 ; -5.480654e+01 ; 5.753284e+02 ];
omc_error_3 = [ 2.129730e-02 ; 1.310753e-02 ; 2.462336e-02 ];
Tc_error_3  = [ 1.294893e+01 ; 1.023766e+01 ; 5.995120e+00 ];

%-- Image #4:
omc_4 = [ -1.689756e+00 ; -1.391230e+00 ; 3.998855e-01 ];
Tc_4  = [ 2.659443e+01 ; -1.117290e+02 ; 5.490922e+02 ];
omc_error_4 = [ 1.659953e-02 ; 1.496256e-02 ; 2.105550e-02 ];
Tc_error_4  = [ 1.230203e+01 ; 9.718282e+00 ; 6.561897e+00 ];

%-- Image #5:
omc_5 = [ -1.732019e+00 ; -2.251872e+00 ; 7.370417e-01 ];
Tc_5  = [ 2.344934e+02 ; -6.039384e+01 ; 6.889756e+02 ];
omc_error_5 = [ 1.637178e-02 ; 1.711558e-02 ; 2.838371e-02 ];
Tc_error_5  = [ 1.605994e+01 ; 1.249452e+01 ; 8.472781e+00 ];

%-- Image #6:
omc_6 = [ -1.685499e+00 ; -2.446391e+00 ; 7.019082e-01 ];
Tc_6  = [ 2.196322e+02 ; -1.033602e+02 ; 7.337378e+02 ];
omc_error_6 = [ 1.639385e-02 ; 1.519940e-02 ; 3.421253e-02 ];
Tc_error_6  = [ 1.681205e+01 ; 1.327884e+01 ; 8.774951e+00 ];

%-- Image #7:
omc_7 = [ -1.676574e+00 ; -1.849929e+00 ; 1.110150e+00 ];
Tc_7  = [ 1.744923e+02 ; -1.541633e+02 ; 6.841604e+02 ];
omc_error_7 = [ 1.920930e-02 ; 9.660755e-03 ; 2.168207e-02 ];
Tc_error_7  = [ 1.597634e+01 ; 1.211049e+01 ; 9.279645e+00 ];

%-- Image #8:
omc_8 = [ -1.422884e+00 ; -1.330164e+00 ; 6.670593e-01 ];
Tc_8  = [ 9.192781e+01 ; 3.780490e+01 ; 6.063869e+02 ];
omc_error_8 = [ 1.648004e-02 ; 1.750664e-02 ; 1.667510e-02 ];
Tc_error_8  = [ 1.371704e+01 ; 1.072890e+01 ; 7.851454e+00 ];

%-- Image #9:
omc_9 = [ -1.788301e+00 ; -1.353100e+00 ; 9.853370e-02 ];
Tc_9  = [ -3.930055e+00 ; -7.737436e+01 ; 5.414801e+02 ];
omc_error_9 = [ 1.582545e-02 ; 1.636800e-02 ; 2.197705e-02 ];
Tc_error_9  = [ 1.202073e+01 ; 9.577256e+00 ; 6.345423e+00 ];

