		function [y, varargout] = output(obj, u)
%
%  OUTPUT: Returns value of the output variables 
%  ==============================================
%  
%  
%  SYNTAX
%  ------
%     
%      y = system.output()
%      y = system.output(u)
%    
%  
%  DESCRIPTION
%  -----------
%     This function evaluates the system's output equation and returns the
%  calculated output. For systems with direct feed-through, it is necessary to
%  provide the vector of inputs as the first input argument.
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
%          y Vector of outputs                        
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
 
 
			% Evaluates the output equation
			
			x = obj.getValues('x');
			if isempty(x)
				error('Internal state not set, use "sys.initialize(x0)".');
			end
			
			if nargin==1 && obj.has_feedthrough
				error('Input is required for systems with direct feed-through.')
			end
			if nargin<2
				u = zeros(obj.nu, 1);
			end
			u = obj.validateInput(u);
			
            varargout = cell(1, nargout-1);
			[y, varargout{:}] = obj.output_equation(x, u);
		end
