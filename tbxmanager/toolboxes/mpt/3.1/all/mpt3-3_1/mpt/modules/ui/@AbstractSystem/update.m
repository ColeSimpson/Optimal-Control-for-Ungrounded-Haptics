		function [xn, y, varargout] = update(obj, u)
%
%  UPDATE: Updates the internal state using the state-update equation 
%  ===================================================================
%  
%  
%  SYNTAX
%  ------
%     
%      system.update(u)
%      xn = system.update(u)
%      [xn, y] = system.update(u)
%    
%  
%  DESCRIPTION
%  -----------
%     This function evaluates the system's state-update equation and updates the
%  internal state of the system.
%    By calling system.update(u) this function updates the internal state of system
%  by evaluating the state-update equation. The updated state can then be retrieved
%  by calling x = system.getStates().
%    By calling [xn, y] = system.update(u) this function also returns the updated
%  state as the first output, and the result of the output equation as the second
%  output. Note that the internal system's state is still updated as described
%  above.
%  
%  INPUT
%  -----
%     
%        
%          u Vector of system's inputs                
%            Class: double                            
%              
%  
%  
%  OUTPUT
%  ------
%     
%        
%          xn Updated state vector                     
%             Class: double                            
%          y  Vector of outputs                        
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
 
 
			% xn = obj.update(u)
			% [xn, y] = obj.update(u)
			% [xn, y, z, d] = obj.update(u)
			
			if nargin<2
				u = [];
			end
			u = obj.validateInput(u);
			
			x = obj.getValues('x');
			if isempty(x)
				error('Internal state not set, use "sys.initialize(x0)".');
			end
			% update_equation() always returns at least "x" and "y", but
			% can return more for MLD systems
			varargout = cell(1, nargout-2);
			[xn, y, varargout{:}] = obj.update_equation(x, u);
			
			% update the internal state
            obj.initialize(xn);
		end
