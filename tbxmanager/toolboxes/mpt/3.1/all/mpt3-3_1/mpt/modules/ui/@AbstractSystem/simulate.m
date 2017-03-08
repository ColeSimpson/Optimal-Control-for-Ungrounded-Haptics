		function out = simulate(obj, U)
%
%  SIMULATE: Simulates evolution of the system 
%  ============================================
%  
%  
%  SYNTAX
%  ------
%     
%      data = system.simulate(U)
%    
%  
%  DESCRIPTION
%  -----------
%     data = system.simulate(U) computes evolution of system's states and outputs,
%  starting from the system's internal state and using a sequence of inputs U.
%    Each column of U is interpreted as the control action to use at the m-th step
%  of the simulation. The total number of simulation steps is given by the number
%  of columns of U. To simulate an autonomous system over Msteps, you need to
%  define U=zeros(0, M).
%    This function returns a structure data, which contains the simulated evolution
%  of system's states (in data.X) and the outputs (in data.Y), respectively.
%    Note that you should always run system.initialize(x0) to set the initial
%  condition prior to running the simulation.
%    Also note that the simulate method updates the internal system's state.
%  Therefore once the function completes, the internal state will be set to the
%  final value obtained at the end of the simulation.
%  
%  INPUT
%  -----
%     
%        
%          U Matrix of control inputs stored          
%            column-wise for each simulation step.    
%            Class: double                            
%              
%  
%  
%  OUTPUT
%  ------
%     
%        
%          data Structure with simulated state and       
%               output profiles.                         
%               Class: struct                            
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
			% Simulates the system using a given sequence of inputs
			%
			
			nu = obj.nu;
			if nu>0 && nargin==1
				error('System is not autonomous, you must provide sequence of inputs as well.');
			end
			Nsim = size(U, 2);
			
			X = obj.getStates(); Y = [];
			if isempty(X)
				error('Set the initial condition first.');
			end
			if size(U, 1) ~= nu
				error('The input sequence must have %d rows.', nu);
			end
			
			for k = 1:Nsim
				[x, y] = obj.update(U(:, k));
				X = [X x];
				Y = [Y y];
			end
			out.X = X;
			out.U = U;
			out.Y = Y;
		end
