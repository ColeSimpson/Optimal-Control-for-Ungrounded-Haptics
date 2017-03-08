        function EMPC = toExplicit(obj)
%
%  TOEXPLICIT: Computes the explicit solution to the MPC optimization problem 
%  ===========================================================================
%  
%  
%  SYNTAX
%  ------
%     
%      expctrl = controller.toExplicit()
%    
%  
%  DESCRIPTION
%  -----------
%     expctrl = controller.toExplicit() computes the explicit solution to the MPC
%  optimization problem defined by the controller object and returns an instance of
%  a class representing explicit MPC controllers.
%    The generated explicit controller inherits all methods and properties of the
%  input object.
%  
%  SEE ALSO
%  --------
%     EMPCController
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
 
 
			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
            EMPC = EMPCController;
			EMPC.model = obj.model;
			EMPC.N = obj.N;
			
			% propagate custom YALMIP setup if provided
			if ~isempty(obj.yalmipData)
				% use YALMIP data to construct the explicit solution
				EMPC.fromYALMIP(obj.yalmipData);
			else
				% construct the explicit solution
				EMPC.construct();
			end
		end
