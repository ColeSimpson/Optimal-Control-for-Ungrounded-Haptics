		function P = LQRPenalty(obj)
%
%  LQRPENALTY: Linear-quadratic regulator design for LTI systems 
%  ==============================================================
%  
%  
%  SYNTAX
%  ------
%     
%      P = system.LQRPenalty()
%    
%  
%  DESCRIPTION
%  -----------
%     P = system.LQRPenalty() calculates the solution of the discre-time Riccati
%  equation.
%    Note that this function requires that penalties on states and inputs are
%  defined in system.x.penalty and system.u.penalty (see " help SystemSignal" and "
%  help Function").
%  
%  OUTPUT
%  ------
%     
%        
%          P Solution of the discrete-time Riccati    
%            equation                                 
%            Class: double                            
%              
%  
%  

%  AUTHOR(s)
%  ---------
%     
%    
%   (c) 2003-2013  Michal Kvasnica: STU Bratislava
%   mailto:michal.kvasnica@stuba.sk 
%  
%  

%  LICENSE
%  -------
%    
%    This program is free software; you can redistribute it and/or modify it under
%  the terms of the GNU General Public License as published by the Free Software
%  Foundation; either version 2.1 of the License, or (at your option) any later
%  version.
%    This program is distributed in the hope that it will be useful, but WITHOUT
%  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
%  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
%    You should have received a copy of the GNU General Public License along with
%  this library; if not, write to the  Free Software Foundation, Inc.,  59 Temple
%  Place, Suite 330,  Boston, MA 02111-1307 USA
%  -------------------------------------------------------------------------------
%    
%      This document was translated from LaTeX by HeVeA (1).
%  ---------------------------------------
%    
%    
%   (1) http://hevea.inria.fr/index.html
 
 
			% Returns the LQR penalty
			if nnz(obj.f)>0 || nnz(obj.g)>0
				error('This function does not support affine systems.');
			end
			error(obj.assert_has_xu_penalties);
			[~, Q] = dlqr(obj.A, obj.B,	obj.x.penalty.weight, obj.u.penalty.weight);
			P = QuadFunction(Q);
		end
