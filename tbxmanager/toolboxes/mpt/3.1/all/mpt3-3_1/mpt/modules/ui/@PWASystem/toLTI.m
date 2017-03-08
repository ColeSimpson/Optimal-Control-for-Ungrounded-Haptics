		function out = toLTI(obj, pwa_index)
%
%  TOLTI: Converts the i-th mode of a PWA system to an LTI system 
%  ===============================================================
%  
%  
%  SYNTAX
%  ------
%     
%      ltisys = pwasys.toLTI(index)
%    
%  
%  DESCRIPTION
%  -----------
%     For a PWA system consisting of M  local affine models, this function converts
%  the index-th model to an instance of the LTISystem class.
%  
%  INPUT
%  -----
%     
%        
%          index Index of the local model to extract.     
%                Class: double                            
%                  
%  
%  
%  OUTPUT
%  ------
%     
%        
%          ltisys LTI representation of the i-th local     
%                 model.                                   
%                 Class: LTISystem                         
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
 
 
			% Converts dynamics indexed by "pwa_index" to an LTI system
			
			if pwa_index<1 || pwa_index>obj.ndyn
				error('Index out of range.');
			end
			out = obj.modes(pwa_index);
		end
