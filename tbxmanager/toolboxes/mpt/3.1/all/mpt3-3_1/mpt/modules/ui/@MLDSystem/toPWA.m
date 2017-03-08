        function out = toPWA(obj)
%
%  TOPWA: Converts the MLD model into an equivalent PWA form 
%  ==========================================================
%  
%  
%  SYNTAX
%  ------
%     
%      pwasys = mldsys.toPWA()
%    
%  
%  DESCRIPTION
%  -----------
%     For a PWA system consisting of M  local affine models, this function converts
%  the index-th model to an instance of the LTISystem class.
%  
%  OUTPUT
%  ------
%     
%        
%          pwasys PWA representation of the MLD system.    
%                 Class: PWASystem                         
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
 
 
            sysStruct = mpt_pwa2sys(hys2pwa(obj.S), obj.S);
			
			% TODO: check that we correctly propagate binarity of states,
			% inputs and outputs
            out = PWASystem(sysStruct);
        end
