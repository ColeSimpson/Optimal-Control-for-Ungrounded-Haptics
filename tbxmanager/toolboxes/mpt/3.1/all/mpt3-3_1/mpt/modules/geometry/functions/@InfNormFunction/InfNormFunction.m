classdef InfNormFunction < NormFunction
%
%  INFNORMFUNCTION: Class representing inf-norm function. 
%  =======================================================
%  
%  
%  SYNTAX
%  ------
%     
%      f = InfNormFunction(Q)
%    
%  
%  DESCRIPTION
%  -----------
%     The object for representing infinity-norm function given as f=||Qx||_oo. The
%  function is given as a maximum absolute value over elements of the vector y=Qx. 
%                                                     
%                              f = max(|y |,...,|y |) 
%                                        1        n   
%     where n  is the dimension of the vector y. The weight Qdoes not need to be
%  square. Function value is always scalar.
%  
%  
%  INPUT
%  -----
%     
%        
%          Q Weighing matrix where the number of      
%            columns determines the dimension of the  
%            vector argument.                         
%            Class: double                            
%              
%  
%  
%  OUTPUT
%  ------
%     
%        
%          f The InfNormFunction object.              
%            Class: InfNormFunction                   
%              
%  
%  
%  SEE ALSO
%  --------
%     OneNormFunction,  AffFunction,  QuadFunction
%  

%  AUTHOR(s)
%  ---------
%     
%    
%   (c) 2003-2013  Michal Kvasnica: STU Bratislava
%   mailto:michal.kvasnica@stuba.sk 
%     
%    
%   (c) 2010-2013  Martin Herceg: ETH Zurich
%   mailto:herceg@control.ee.ethz.ch 
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
 
 
	%
	% represents weighted inf-norm functions
	%
	% syntax:
	%   f = InfNormFunction(Q) : f = norm(Q*x, Inf)
	%
	% "Q" need not to be square. Function value is always scalar.
	
	methods
		
		% Constructor
		function obj = InfNormFunction(Q)
			% Constructs a weighted 1-norm function object
			%
			% syntax:
			%   f = InfNormFunction(Q) : f = norm(Q*x, Inf)
			%
			% "Q" need not to be square. Function value is always scalar.
			
			error(nargchk(1,1,nargin));
			obj = obj@NormFunction(Inf, Q);
		end
		
	end
	
end
