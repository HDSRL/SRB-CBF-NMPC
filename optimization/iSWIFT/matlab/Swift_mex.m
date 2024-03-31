function Swift_mex(file)

curpath = cd;
curpath = curpath(1:end-6); % remove matlab from the curpath
if(isempty(file))
    disp('Enter the name of the s_function ot be used');
else
d = '-largeArrayDims -DDLONG -DLDL_LONG';
%% ldl solver
    fprintf('Compiling LDL...\n');
    cmd = ['mex -c -O -largeArrayDims ','-I',curpath,'ldl\include ',curpath,'ldl\src\ldl.c'];
    eval(cmd);
    fprintf('LDL Compilation Done\n');
%% QP (core IPM solver)
    fprintf('Compiling QP...\n');
    cmd = ['mex -c -O -largeArrayDims ','-I',curpath,'\include ','-I',curpath,'ldl\include ',curpath,'src\Auxilary.c ',curpath,'src\Prime.c ',curpath,'src/timer.c '];
    eval(cmd);
    fprintf('QP Compilation Done !!!\n');
 
%% QP_mex
    fprintf('Compiling QP_sfunc...\n');
    cmd = ['mex -g -c -O -largeArrayDims ','-I',curpath,'\include ','-I',curpath,'ldl\include ',file,'.c'];
    eval(cmd);
    fprintf('QP_sfunc Compilation Done\n');
    fprintf('Linking Object Files...\n');
    cmd = ['mex -largeArrayDims ','Prime.obj Auxilary.obj timer.obj ldl.obj ',file '.obj',' -output ', file];
    eval(cmd);
    fprintf('Successfully Compiled \n');

%% clean
    if( ispc ), delete('*.obj'); end
    fprintf('Object Files cleaned \n');
    clear;
end
    