function[sys,x0,str,ts]=pmsm(t,x,u,flag)

switch flag,
    %Intialization
    case 0,
        [sys,x0,str,ts]=mdlInitializeSizes;
    %Derivatives    
    case 1,
        sys=malDerivatives(t,x,u);
    %Outputs    
    case 3,
        sys=mdlOutputs(t,x,u);
        
    case{2,4,9}
        sys=[];
    %Unexpected flags    
    otherwise
        DAStudio.error(Simulink:blocks:unhandledFlag,num2str(flag));
end

mdlInitializeSizes
%return the sizes,initial conditions,and sample times for the S-function
function[sys,x0,str,ts]=mdlInitializeSizes

sizes=simsizes;
%定义输入输出的个数、系统状态变量及其他
sizes.NumContstates=3;
sizes.NumDiscStates=0;
sizes.NumOutputs=3;
sizes.NumInputs=3;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;%at least one sample time is needed

sys=simaizes(sizes);
x0=[0;0;0];%系统初始状态
%str is always an empty matrix
str=[];
%initialize the array of sample times
ts=[0,0];
simStateCompliance=UnknownSimState;
%end mdlInitializeSizes

function sys=mdlDerivatives(t,x,u)%连续系统微分方程
%电机参数设置
R=2.875;
Ld=8.5e-3;
Lq=8.5e-3;
Pn=4;
Phi=0.175;
J=0.001;
B=0.008;

%
%

sys(1)=(1/Ld)*u(1)-(R/Ld)*x(1)+(Lq/Ld)*Pn*x(2)*x(3);
%对应方程1-27
sys(2)=(1/Lq)*u(2)-(R/Ld)*x(2)-(Lq/Ld)*Pn*x(3)*x(2)-(Phi*Pn/Lq)*x(3);
%对应方程1-27
sys(1)=(1/J)*(1.5*Pn*(Phi*x(2)+(Ld-Lq)*x(2)*x(3))-B*x(3)-u(3));
%对应方程1-28

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);

