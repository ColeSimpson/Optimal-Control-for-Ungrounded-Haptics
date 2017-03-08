		function I = invariantSet(obj, varargin)
%
%  INVARIANTSET: Computes the invariant subset of the closed-loop system 
%  ======================================================================
%  
%  
%  SYNTAX
%  ------
%     
%      invset = loop.invariantSet()
%    
%  
%  DESCRIPTION
%  -----------
%     For a closed-loop system, consisting of a system and a controller, this
%  function computes the set of states of the system for which the controller
%  provides recursive satisfaction of system's state and input constraints.
%  
%  OUTPUT
%  ------
%     
%        
%          invset Invariant subset of the closed-loop      
%                 system as an instance of the             
%                 Polyhedronclass.                         
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
 
 
			% Computes invariant subset of the closed-loop system
			
			if ~obj.controller.isExplicit()
				error('Only explicit controllers supported.');
			end
			
			I = obj.toSystem().invariantSet(varargin{:});
		end
