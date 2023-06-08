pkg load control
clear all
close all
format short g

                            %Parametros do Sistema

rb = 0.054/2                %Raio da Bola.
rw= 0.150-rb;               %Raio da Roda.
mw= 0.400;                  %Massa da Roda.
mb = 0.045;                 %Massa da Roda.
Iw= 0.001;                  %Inercia da Roda.
Ib= ((2/3)*mb*(rb^2));      %Inercia da Bola.
km= 0.069546;               %Constante do Motor.
ra= 3.767926;               %Resistencia de Armadura.
g=  9.80665;                %Gravidade.
l = 0.150;                  %Raio entre os centros da Roda e da Bola.

T=0.02;
t = 0:T:20;
refer=0.0;
x0 = [0;0;0;0];
s = tf('s');

a= (-((2*rw*(km^2))/(ra*(7*Iw+2*(rw^2)*mb)*(rb+rw))));
b= ((g*(5*Iw+2*(rw^2)*mb))/((7*Iw+2*(rw^2)*mb)*(rb+rw)));
c= (2*rw*km)/(ra*((7*Iw+ (2*(rw^2)*mb))*(rb+rw)));
p= (-(7*(km^2))/(ra*(7*Iw+2*(rw^2)*mb)));
q= (2*g*rw*mb)/(7*Iw+2*(rw^2)*mb);
r= (7*km)/(ra*(7*Iw+2*(rw^2)*mb));


Am = [ 0 1 0 0; b 0 0 a; 0 0 0 1; q 0 0 p]; #Matriz A Sem Mudar Posição.

Bm = [0; c; 0; r];                          #Matriz B Sem Mudar Posição.

Cm = [1,0,0,0;0,0,1,0];                     #Matriz C Sem Mudar Posição.

A = [ 0 0 1 0; 0 0 0 1; b 0 0 a ; q 0 0 p]; #Matriz A.

B = [0; 0; c; r];                           #Matriz B.

C = [1,0,0,0;0,1,0,0];                      #Matriz C.

stname = {'AngBola', 'AngMotor', 'VelBola','VelMotor'};
sys = ss(A,B,C,'stname',stname);            %Sistema Continuo;
sysantigo = ss(Am,Bm,Cm,'stname',stname);   %Sistema Continuo;

R = 1;
Q = diag([1,0.000001,10,0.000001]);
tempo = 5;

k = lqr(A,B,Q,R)
sysC = ss(A-B*k,B*k(1),C);                  %Sistema Continuo com Controle.
u_impulse = zeros(1,tempo/T+1);
u_impulse(1) = 1
[y,t,x] = lsim(sysC, 2*u_impulse,tempo);
u = -(k(1)*x(:,1)+k(2)*x(:,2)+k(3)*x(:,3)+k(4)*x(:,4));

sysD = c2d(sys,T);                          %Sistema Discreto.
Ad = sysD.a;
Bd = sysD.b;
Cd = sysD.c;

F = Q;
for i =1:100000
  W = R + Bd'*F*Bd;
  P = F - F*Bd*(W^-1)*Bd'*F;
  F = Ad'*P*Ad+Q;
endfor
k1 = (W^-1)*Bd'*F*Ad;

sysDC = ss(Ad-Bd*k1,Bd*k1(1),sys.c,0,T,'stname',stname); %Sistema Discreto com Controle.
[y1, t1, x1] = lsim(sysDC, u_impulse,tempo);
u1 = -(k1(1)*x1(:,1)+k1(2)*x1(:,2)+k1(3)*x1(:,3)+k1(4)*x1(:,4)); %Ação de Controle.

%%Observador de Ordem Plena.
tal = [0 0 0 0]; %Constante de Tempo.
tal(1) = -T/log(eig(Ad-Bd*k1)(1));
tal(2) = -T/log(eig(Ad-Bd*k1)(2));
tal(3) = -T/log(eig(Ad-Bd*k1)(3));
tal(4) = -T/log(eig(Ad-Bd*k1)(4));
tal_menor = min(tal);
p4 = exp(-T/abs(tal_menor));
talp4 = -T/log(p4);

p = [p4 p4 p4 p4];

L = place(Ad',Cd',p)';

L1=(Ad - L*Cd);

##%Observador de Ordem Reduzida.
Amm = Ad(1:2,1:2);
Amo = Ad(1:2,3:4);
Aom = Ad(3:4,1:2);
Aoo = Ad(3:4,3:4);
Bm = Bd(1:2);
Bo = Bd(3:4);

A1o = Aoo;
C1o = Amo;

tal_min = [0 0 0 0]; %Constante de Tempo.
tal_min(1) = -T/log(eig(Ad-Bd*k1)(1));
tal_min(2) = -T/log(eig(Ad-Bd*k1)(2));
tal_min(3) = -T/log(eig(Ad-Bd*k1)(3));
tal_min(4) = -T/log(eig(Ad-Bd*k1)(4));
tal_menor_min = min(tal_min);
p4_min = exp(-T/abs(tal_menor_min/2));
talp4_min = -T/log(p4_min);
p_min = [p4_min p4_min];

L_min = place(A1o',C1o',p_min)';
L1_min = Aoo-L_min*Amo;
B_min = (Bo-L_min*Bm);
A_min = (Aom-L_min*Amm);

IMPULSE = 30;

%Simulação do Observador Pleno.
x2 = x0;
x2O = zeros(4,tempo/T);
u2O = zeros(1,tempo/T);
x2O(:,1) = x0
u2O(1) = IMPULSE-(k1(1)*x2O(1,1)+k1(2)*x2O(2,1)+k1(3)*x2O(3,1)+k1(4)*x2O(4,1));

for i = 2:(tempo/T+1)
x2(:,i) = Ad*x2(:,i-1) + Bd*u2O(i-1);
y2(:,i) = Cd*x2(:,i);

x2O(:,i) = L1*x2O(:,i-1)+Bd*u2O(:,i-1)+L*y2(:,i);
u2O(i) = -(k1(1)*x2O(1,i)+k1(2)*x2O(2,i)+k1(3)*x2O(3,i)+k1(4)*x2O(4,i));

%Saturação do Motor. 
if(u2O(i)>(24))
u2O(i) = (24);
endif
if(u2O(i)<-(24))
u2O(i) = -(24);
endif



endfor


%Simulação do Observador Minimo
x3 = x0;
x3O = zeros(2,tempo/T);
u3O = zeros(1,tempo/T);
u3O(1) = IMPULSE-(k1(1)*x'(1,1)+k1(2)*x'(2,1)+k1(3)*x3O(1,1)+k1(4)*x3O(2,1));

for i = 2:(tempo/T+1)
x3(:,i) = Ad*x3(:,i-1) + Bd*u3O(i-1);
y3(:,i) = Cd*x3(:,i);

x3O(:,i) = L1_min*x3O(:,i-1)+B_min*u3O(:,i-1)+L_min*y3(:,i)+A_min*(y3(:,i-1));
u3O(i) = -(k1(1)*y3(1,i)+k1(2)*y3(2,i)+k1(3)*x3O(1,i)+k1(4)*x3O(2,i));

%Saturação do Motor. 
if(u3O(i)>(24))
u3O(i) = (24);
endif
if(u3O(i)<-(24))
u3O(i) = -(24);
endif



endfor

%Comparação do Observador.
x4 = x0;
x4O = zeros(4,tempo/T);
x4O(:,1) = x0
x4O_min = zeros(2,tempo/T);
u4 = zeros(1,tempo/T);
u4(1) = -(k1(1)*x4(1,1)+k1(2)*x4(2,1)+k1(3)*x4(3,1)+k1(4)*x4(4,1));
if(u4(1)>(24))
u4(1) = (24);
endif
if(u4(1)<-(24))
u4(1) = -(24);
endif
for i = 2:(tempo/T+1)
x4(:,i) = Ad*x4(:,i-1) + Bd*u4(i-1);
y4(:,i) = Cd*x4(:,i);

x4O(:,i) = L1*x4O(:,i-1)+Bd*u4(:,i-1)+L*y4(:,i);
x4O_min(:,i) = L1_min*x4O_min(:,i-1)+B_min*u4(:,i-1)+L_min*y4(:,i)+A_min*(y4(:,i-1));
u4(i) = -(k1(1)*x4(1,i)+k1(2)*x4(2,i)+k1(3)*x4(3,i)+k1(4)*x4(4,i));

%Saturação do Motor.
if(u4(i)>(24))
u4(i) = (24);
endif
if(u4(i)<-(24))
u4(i) = -(24);
endif
endfor

t1 = t1(2:251)
t = t(2:251)
x = x(2:251,:)
x1 = x1(2:251,:)
x2 = x2(:,2:251)
x2O = x2O(:,2:251)
x3 = x3(:,2:251)
x3O = x3O(:,2:251)
x4 = x4(:,2:251)
x4O = x4O(:,2:251)
x4O_min= x4O_min(:,2:251)
u = u(2:251)
u1 = u1(2:251)
u2O = u2O(2:251)
u3O = u3O(2:251)

#Resposta ao Impulso do Sistema Continuo.k
figure
e = tf(sys)
[y,t,x] = impulse((e(1,1)),0:0.02:0.4)
plot(t,rad2deg(x(:,1)))
xlabel("Tempo(s)")
ylabel("Ângulo(Graus)")


Aoc = (Aoo-L*Amo)
Boc=(Bo-L*Bm)
L1c = L
L2c = Aom-L*Amm

k1
Aa = Aoc
Bp = Boc
Ke = L
Ab = L2c

k1
L1
Bd'
L

k1
L1_min
B_min'
L_min
A_min

