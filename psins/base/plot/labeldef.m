function stext = labeldef(stext)
% Define special labels for conciseness.
% 
% Prototype: stext = labeldef(stext)
% Input: stext - a short text input
% Output: stext - corresponding fully formated text output
%
% See also  xygo.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/03/2014
    specl = {...  % string cell
        'phi',  '\it\phi\rm / \prime';
        'phiE',  '\it\phi_E\rm / \prime\prime';
        'phiN',  '\it\phi_N\rm / \prime\prime';
        'phiU',  '\it\phi_U\rm / \prime';
        'phiEN', '\it\phi _{E,N}\rm / \prime\prime';
        'phiz',  '\it\phi_z\rm / \circ';
        'phixy', '\it\phi _{x,y}\rm / \circ';
        'mu',    '\it\mu \rm / \prime';
        'mux',    '\it\mu_x \rm / \prime';
        'muy',    '\it\mu_y \rm / \prime';
        'muz',    '\it\mu_z \rm / \prime';
        'theta', '\it\theta \rm / \prime';
        'dVEN',  '\it\delta V _{E,N}\rm / m/s';
        'dVE',   '\it\delta V _E\rm / m/s';
        'dVN',   '\it\delta V _N\rm / m/s';
        'dVU',   '\it\delta V_U\rm / m/s';
        'dV',    '\it\delta V\rm / m/s';
        'pr',    '\it\theta , \gamma\rm / \circ';
        'ry',    '\it\gamma , \psi\rm / \circ';
        'p',     '\it\theta\rm / \circ';
        'r',     '\it\gamma\rm / \circ';
        'y',     '\it\psi\rm / \circ';
        'att',   '\itAtt\rm / \circ';
        'VEN',   '\itV _{E,N}\rm / m/s';
        'VU',    '\it V_U\rm / m/s';
        'V',     '\it V\rm / m/s';
        'Vx',     '\it Vx\rm / m/s';
        'Vy',     '\it Vy\rm / m/s';
        'Vz',     '\it Vz\rm / m/s';
        'dlat',  '\it\delta L\rm / m';
        'dlon',  '\it\delta \lambda\rm / m';
        'dH',    '\it\delta H\rm / m';
        'dP',    '\it\delta P\rm / m';
        'lat',   '\itL\rm / \circ';
        'lon',   '\it\lambda\rm / \circ';
        'est',   '\itEast\rm / m';
        'nth',   '\itNorth\rm / m';
        'H',     '\itH\rm / m';
        'DP',    '\it\Delta P\rm / m';
        'ebyz',    '\it\epsilon _{y,z}\rm / (\circ/h)';
        'eb',    '\it\epsilon\rm / (\circ/h)';
        'db',    '\it\nabla\rm / ug';
        'dbU',   '\it\nabla _U\rm / ug';
        'L',     '\itLever\rm / m';
        'dT',    '\it\delta T_{asyn}\rm / s';
        'dKg',   '\it\delta Kg\rm / ppm';
        'dAg',   '\it\delta Ag\rm / \prime\prime';
        'dKa',   '\it\delta Ka\rm / /ppm';
        'dAa',   '\it\delta Aa\rm / \prime\prime';
		'wx',    '\it\omega_x\rm / (\circ/s)';
		'wy',    '\it\omega_y\rm / (\circ/s)';
		'wz',    '\it\omega_z\rm / (\circ/s)';
		'w',     '\it\omega\rm / (\circ/s)';
		'fx',    '\itf_x\rm / g';
		'fy',    '\itf_y\rm / g';
		'fz',    '\itf_z\rm / g';
		'f',     '\itf\rm / g';
		'dinst', '\it\delta\theta , \delta\psi\rm / \prime';
    };
    for k=1:size(specl,1)
        if strcmp(stext,specl(k,1))==1
            stext = specl{k,2};
            break;
        end
    end