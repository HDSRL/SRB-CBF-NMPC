clc;
clear all;
close all;

%%% Generates the permutation matrix for given P,A and G matrices and 
%%% prints the output in myfile.txt

load('Matrices.mat');

n = size(P,1);
m = size(G,1);
p = size(A,1);

Phi = [P A' G'; A zeros(p ,m + p);G zeros(m,p) -eye(m,m)];

Permut = amd(Phi) - 1;



dlmwrite('myfile.txt',['idxint Permut[',num2str(n+m+p),'] = {'],'delimiter','');
dlmwrite('myfile.txt',Permut,'-append');
dlmwrite('myfile.txt','};','-append','delimiter', '','newline','pc');