%% Jessica De la Cruz
%% UAM RESEARCH

clc 
clear all

%% A procedure was developed to observe trends of SSTOL performance with inputs of an operating CL and corresponding velocity.
%% Flight paths angles for T/O and LDG are calculated for a vehicle with a blowing wing

%Interest points:
%Takeoff departure: Full throttle, 18 kts, 20 deg flight path angle, 25 deg flaps.
%Landing approach: 50% throttle, 16 kts, -10 deg flight path angle, 40 deg flaps.
%Overflight: 100% throttle, 60 kts, 0 deg flight path angle, 0 flaps

% gamma_to=20;                %[degrees]
% gamma_app=-10;
% V_operating=18*1.69;    %[ft/s]
% V_overflight=60*1.69;   %[ft/s]
% V_landing=16*1.69;  %[ft/s]

%Control Parameters
pulse_width=1800;   %[us]
inner_throttle=1;
outer_throttle=0;
h=400; %[ft]
AOA=0;

%Geometry
D=7/12;                 %[ft]
R=D/2;
d=1/12;                 %[ft]
r=d/2;
n_prop=8;
b=156/12;               %[ft]
c=18/12;                %[ft]
S=18.33;                %[ft^2]
W=38.9;                %[lbs]
g=32.174;               %[ft/s^2]
P=550;                 %[Watts]
A=pi*R^2;               %[ft^2]
rho_SL=0.002377;
t_roll=3; %[sec]

%Parameters
roll_coefficient=0.05;
CDP=0.045;
f=S*CDP;                %[ft^2]
AR=b^2/S;
e=1.78*(1-0.045*AR^0.68)-0.64;

CL=[2,4,6]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:length(CL)

%T/O
V_operating=sqrt(2*W/(rho_SL*S*CL(i))); 
% Vto=1.2*V_operating;
Vto=V_operating;
% Vto_7=0.7*Vto;
Vto_7=V_operating;

% Thrust
% 75% throttle
RPM=0.00242*(pulse_width)^2 + 11.526*(pulse_width) - 14432;
RPM=0.75*RPM*((2*pi())/60); %[rad/S]

advance_ratio=Vto_7/(RPM*R);

CT= -0.0838*(advance_ratio)^2 - 0.1179*(advance_ratio) + 0.0652;

T_engine_blowing=CT*0.5*rho_SL*((R*RPM)^2)*A; 
hd_c=(pi*(R^2-r^2)*n_prop)/(b*c);
Vj=sqrt((2*T_engine_blowing)/(rho_SL*A)+Vto_7^2);

% AT 100 percent Throttle
T_total= T_engine_blowing*n_prop;

ratio=Vj/Vto_7;
delta_CJ=hd_c*(ratio^2-1)*((1/ratio)+1);

%% Aerodynamics
VD=(Vj+Vto_7)/2;
hj=hd_c*c*(VD/Vj);
CJ=2*(ratio)^2*(hj/c);
CT_formula=2*(hj/c)*(ratio^2-ratio);
fb=0.7; 

CL_roll=0.1;  
alpha_i=CL_roll/(pi*AR*e);

%Drag
CDt=2*CJ*fb*alpha_i; 
CDi=CL_roll^2/(pi*AR*e);
CDtotal=CDi+CDP+CDt;

D=CDtotal*0.5*rho_SL*Vto_7^2*S;

% Lift
L=CL_roll*0.5*rho_SL*Vto_7^2*S;

% Rolling friction
R1=roll_coefficient*(W-L);
T=T_total;
a=(T-D-R1)/(W/g); 
Sg=Vto^2/(2*a);

Cx=(D-T)/(0.5*rho_SL*Vto_7^2*S);

if (Sg<=100 && Sg>=0)
    disp("Short Takeoff achieved!");
else
    disp("Cannot Takeoff!");
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Climb
% 75% throttle
RPM_climb=0.00242*(pulse_width)^2 + 11.526*(pulse_width) - 14432;
RPM_climb=0.75*RPM_climb*((2*pi())/60); %[rad/S]
V_climb=sqrt((2*W)/(rho_SL*CL(i)*S));

advance_ratio_climb=V_climb/(RPM*R);

CT_climb= -0.0838*(advance_ratio_climb)^2 - 0.1179*(advance_ratio_climb) + 0.0652;

T_engine_blowing_climb=CT_climb*0.5*rho_SL*((R*RPM)^2)*A; 
T_blowing_climb=T_engine_blowing_climb;
hd_c_climb=(pi*(R^2-r^2)*n_prop)/(b*c);
Vj_climb=sqrt((2*T_blowing_climb)/(rho_SL*A)+V_climb^2);

ratio_climb=Vj_climb/V_climb;
VD_climb=(Vj_climb+V_climb)/2;
hj_climb=hd_c*c*(VD_climb/Vj_climb);
CJ_climb=2*(ratio_climb)^2*(hj_climb/c);
fb=0.7; 

alpha_i_climb=CL(i)/(pi*AR*e);

%Drag
CDt_climb=2*CJ_climb*fb*alpha_i_climb; 
CDi_climb=CL(i)^2/(pi()*AR*e);
CDtotal_climb=CDi_climb+CDP+CDt_climb;
D_climb=CDtotal_climb*0.5*rho_SL*V_climb^2*S;

%sin_gamma_1=(T/W)-(D/W);
gamma_climb=asin(((T_blowing_climb*n_prop)/W)-(D_climb/W));
gamma_climb=gamma_climb*(180/pi());

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Flyover
% L=W, T=D

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Approach/Descent
% 25% thottle
RPM_approach=0.00242*(pulse_width)^2 + 11.526*(pulse_width) - 14432;
RPM_approach=0.25*RPM_approach*((2*pi())/60); %[rad/S]
V_approach=sqrt((2*W)/(rho_SL*CL(i)*S));

% Start approach at 100% throttle
advance_ratio_approach=V_approach/(RPM_approach*R);
CT_approach= -0.0838*(advance_ratio_approach)^2 - 0.1179*(advance_ratio_approach) + 0.0652;


% Outer throttle propellers off
n_prop_approach=6;
T_engine_blowing_approach=CT_approach*0.5*rho_SL*((R*RPM_approach)^2)*A; 
hd_c_approach=(pi*(R^2-r^2)*n_prop_approach)/(b*c);
Vj_approach=sqrt((2*T_engine_blowing_approach)/(rho_SL*A)+V_approach^2);

ratio_approach=Vj_approach/V_approach;
VD_approach=(Vj_approach+V_approach)/2;
hj_approach=hd_c*c*(VD_approach/Vj_approach);
CJ_approach=2*(ratio_approach)^2*(hj_approach/c);
fb=1; 

alpha_i_approach=CL(i)/(pi*AR*e);

%Drag
CDt_approach=2*CJ_approach*fb*alpha_i_approach; 
CDi_approach=CL(i)^2/(pi()*AR*e);
CDtotal_approach=CDi_approach+CDP+CDt_approach;
D_approach=CDtotal_approach*0.5*rho_SL*V_approach^2*S;

%sin_gamma_1=(D/W)-(T/W);
gamma_app=asin((D_approach/W)-((T_engine_blowing_approach*n_prop_approach)/W));
gamma_app=gamma_app*(180/pi());

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LDG

%% Approach
% L_glide=W*cos(deg2rad(gamma_app));
% D_glide=W*cos(deg2rad(gamma_app));
% Sa=50/tan(deg2rad(gamma_app));
% Sfr=Vldg*t_roll;

V_operating=sqrt(2*W/(rho_SL*S*CL(i))); 
Vref=V_operating;
Vldg=V_operating;
Vldg_7=V_operating;

% Thrust
% 25% throttle 
RPM_ldg=0.00242*(pulse_width)^2 + 11.526*(pulse_width) - 14432;
RPM_ldg=0.25*RPM_ldg*((2*pi())/60); %[rad/S]

advance_ratio_ldg=Vldg_7/(RPM_ldg*R);

CT_ldg= -0.0838*(advance_ratio_ldg)^2 - 0.1179*(advance_ratio_ldg) + 0.0652;

T_engine_blowing_ldg=CT_ldg*0.5*rho_SL*((R*RPM_ldg)^2)*A; 
hd_c_ldg=(pi*(R^2-r^2)*n_prop_approach)/(b*c);
Vj_ldg=sqrt((2*T_engine_blowing_ldg)/(rho_SL*A)+Vldg_7^2);

% AT 50% percent Throttle
T_ldg=T_engine_blowing_ldg*n_prop_approach;

ratio_ldg=Vj_ldg/Vldg_7;
delta_CJ_ldg=hd_c_ldg*(ratio_ldg^2-1)*((1/ratio_ldg)+1);

%% Aerodynamics
VD_ldg=(Vj_ldg+Vldg_7)/2;
hj_ldg=hd_c_ldg*c*(VD_ldg/Vj_ldg);
CJ_ldg=2*(ratio_ldg)^2*(hj_ldg/c);
fb=1; 

CL_roll=0.1;  
alpha_i_ldg=CL_roll/(pi*AR*e);

%Drag
CDt_ldg=2*CJ_ldg*fb*alpha_i_ldg; 
CDi_ldg=CL_roll^2/(pi*AR*e);
CDtotal_ldg=CDi_ldg+CDP+CDt_ldg;

D_ldg=CDtotal_ldg*0.5*rho_SL*Vldg_7^2*S;

% Lift
L=CL_roll*0.5*rho_SL*Vldg_7^2*S;

% Rolling friction
% T rev=0
R_ldg=roll_coefficient*(W-L)*10;
a_ldg=(0*T_ldg+D_ldg+R_ldg)/(W/g); 
Sg_ldg=Vldg^2/(2*a_ldg);

% Sg_ldg=Sa+Sfr+Sg_ldg;

Cx_ldg=(D_ldg-T_ldg)/(0.5*rho_SL*Vldg_7^2*S);

if (Sg_ldg<=100 && Sg_ldg>=0)
    disp("Short LDG achieved!");
else
    disp("Cannot LDG!");
end

gamma_results(i)=gamma_climb;
gamma_app_results(i)=gamma_app;
Sg_to_results(i)=Sg;
Sg_ldg_results(i)=Sg_ldg;
Throttle_setting(i)="75% Throttle T/O RPM =" + RPM*(60/(2*pi())) + " | 25% LDG RPM = " + RPM_approach*(60/(2*pi())); %rev/min
Vj_results(i)="Vj T/O ="+Vj/1.69+" | Vj Climb="+Vj_climb/1.69+" | Vj app="+Vj_approach/1.69+" | Vj LDG="+Vj_ldg/1.69; %knots
V_results(i)=V_operating/1.69;

end

Results_matrix=["CL","Velocity [Knots]","Vjs [Knots]","Gamma Climb [Deg]","T/O Sg [ft]","Gamma Approach [Deg]", "LDG Sg [ft]", "Throttle [Rev/Min]"];

table=[1] %% Table for cases
for col = 1:length(table+1)
  for row = 1:length(CL)
      Results_matrix(row+1, col) = CL(row);
      Results_matrix(row+1, col+1) = V_results(row);
      Results_matrix(row+1, col+2) = Vj_results(row);
      Results_matrix(row+1, col+3) = gamma_results(row);
      Results_matrix(row+1, col+4) = Sg_to_results(row);
      Results_matrix(row+1, col+5) = gamma_app_results(row);
      Results_matrix(row+1, col+6) = Sg_ldg_results(row);
      Results_matrix(row+1, col+7) = Throttle_setting(row);
  end
end


%system('"xrotor.exe" input1(string) input2(string)')
