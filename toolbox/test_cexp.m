% -------------------------------------------------------------------------
% TDSÑ-G33 Sesión 2
% Tests para el ejercicio 2
% -------------------------------------------------------------------------
%% Test 1
w0 = pi/2;
n0 = 0;
n = 0:3;
x = cexp(w0, n, n0);   %#ok<*NASGU>
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, [1; 1j; -1; -1j], 'RelTol', 1e-15))
%% Test 2
w0 = pi/2;
n0 = 0;
n = -3:0;
x = cexp(w0, n, n0);
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, [1j; -1; -1j; 1], 'RelTol', 1e-15))
%% Test 3
w0 = pi/2;
n0 = 0;
n = -3:3;
x = cexp(w0, n, n0);
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, [1j; -1; -1j; 1; 1j; -1; -1j], 'RelTol', 1e-15))
%% Test 4
w0 = pi/2;
n0 = 2;
n = -3:3;
x = cexp(w0, n, n0);
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, [-1j; 1; 1j; -1; -1j; 1; 1j], 'RelTol', 1e-15))
%% Test 5
w0 = pi/2;
n0 = -2;
n = -3:3;
x = cexp(w0, n, n0);
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, [-1j; 1; 1j; -1; -1j; 1; 1j], 'RelTol', 1e-15))
%% Test 6
w0 = pi/4;
n0 = -2;
n = -3:3;
x = cexp(w0, n, n0);
assert(exist('x', 'var') == 1)
assert(isalmostequal(x, (cos(w0*(n-n0))+1j*sin(w0*(n-n0))).'), 'RelTol', 1e-15)
%% OK
disp('Todos los tests correctos');