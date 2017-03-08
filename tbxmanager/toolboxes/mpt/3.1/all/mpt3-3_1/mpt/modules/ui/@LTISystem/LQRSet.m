		function S = LQRSet(obj)
%
%  LQRSET: Computes an invariant subset of an LQR controller 
%  ==========================================================
%  
%  
%  SYNTAX
%  ------
%     
%      S = system.LQRSet()
%    
%  
%  DESCRIPTION
%  -----------
%     This function computes the set of states where an LQR controller u=Kx 
%  provides recursive satisfaction of state and input constraints.
%    The set is computed by running the set recursion 
%                                                                     
%                                                                     
%               S    = { x  |  (A+BK) x in S ,  u    <= Kx <= u    }, 
%                k+1                        k    min           max    
%                                                                     
%     initialized by S_0 = { x  |  x_min <= x <= x_max }  and terminating once
%  S_k+1=S_k. The LQR feedback gain is computed automatically by calling K =
%  system.LQRGain().
%    Note that this function requires that input and state constraints are defined
%  in system.u.min, system.u.max, system.x.min, and system.x.max (see " help
%  SystemSignal").
%  
%  OUTPUT
%  ------
%     
%        
%          S Invariant set of the LQR controller      
%            Class: Polyhedron                        
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
 
 
			% Returns the LQR invariant set
			if nnz(obj.f)>0 || nnz(obj.g)>0
				error('This function does not support affine systems.');
			end
			error(obj.assert_has_xu_penalties);
			CL = ClosedLoop(LQRController(obj), obj);
			S = CL.toSystem().invariantSet();
		end
