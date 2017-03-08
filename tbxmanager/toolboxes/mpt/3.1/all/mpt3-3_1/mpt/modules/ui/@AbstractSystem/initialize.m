		function obj = initialize(obj, xinit)
%
%  INITIALIZE: Sets the internal state of the system 
%  ==================================================
%  
%  
%  SYNTAX
%  ------
%     
%      system.initialize(x0)
%    
%  
%  DESCRIPTION
%  -----------
%     system.initialize(x0) sets the initial state of the system to value x0. The
%  internal state can be retrieved by calling system.getStates().
%  
%  INPUT
%  -----
%     
%        
%          x0 Initial state                            
%             Class: double                            
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
 
 
			%
			% Initialize state variables to given values
			%
			
			xinit = xinit(:);
			error(validate_vector(xinit, obj.nx, 'initial state'));
			x = obj.getComponents('x');
			cx = 1;
			for i = 1:length(x)
				if isempty(xinit)
					x.initialize([]);
				else
					x.initialize(xinit(cx:cx+x.n-1));
					cx = cx+x.n;
				end
			end
		end
