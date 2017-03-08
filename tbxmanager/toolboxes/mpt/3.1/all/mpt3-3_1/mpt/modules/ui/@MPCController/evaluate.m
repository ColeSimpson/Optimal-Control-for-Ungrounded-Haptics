        function [u, feasible, openloop] = evaluate(obj, xinit, varargin)
%
%  EVALUATE: Returns the optimal control action 
%  =============================================
%  
%  
%  SYNTAX
%  ------
%     
%      u = controller.evaluate(x0)
%      [u, feasible] = controller.evaluate(x0)
%      [U, feasible, openloop] = controller.evaluate(x0)
%    
%  
%  DESCRIPTION
%  -----------
%     u = controller.evaluate(x0) solves the MPC optimization problem using x0 as
%  the initial condition and returns the first element of the optimal sequence of
%  control inputs. If the problem is infeasible, u will be NaN.
%     [u, feasible] = controller.evaluate(x0) also returns the boolean feasibility
%  flag.
%     [u, feasible, openloop] = controller.evaluate(x0) also returns the open-loop
%  predictions of states, inputs and outputs in openloop.X, openloop.Y, and
%  openloop.Y, respectively. Value of the optimal cost is returned in
%  openloop.cost.
%  
%  INPUT
%  -----
%     
%        
%          x0 Initial state of the MPC optimization    
%             problem.                                 
%             Class: double                            
%               
%  
%  
%  OUTPUT
%  ------
%     
%        
%          u        Optimal control action.                  
%                   Class: double                            
%          feasible True if the optimization problem was     
%                   feasible, false otherwise.               
%                   Class: logical                           
%          openloop Structure containing open-loop           
%                   predictions and value of the optimal     
%                   cost.                                    
%                   Class: struct                            
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
			% u = controller.evaluate(x0) solves the MPC optimization
			% problem using x0 as the initial condition and returns the
			% first element of the optimal sequence of control inputs. If
			% the problem is infeasible, "u" will be NaN.
			%
			% [u, feasible] = controller.evaluate(x0) also returns a
			% boolean flag indicating whether the optimization problem had
			% a feasible solution.
			%
			% [u, feasible, openloop] = controller.evaluate(x0) also
			% returns the open-loop predictions of states, inputs and
			% outputs in "openloop.X", "openloop.Y", and "openloop.Y",
			% respectively. Value of the optimal cost is returned in
			% openloop.cost. 
			%
			% u = controller.evaluate(x0, 'x.reference', ref, 'u.prev', u0)
			% includes "ref" and "u0" into the vector of initial
			% conditions.
			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
			error(validate_vector(xinit, obj.nx, 'initial state'));
			
			if obj.wasModified() || isempty(obj.optimizer)
				% Re-generate the optimizer if the problem setup was
				% changed
				obj.construct();
			end
			% assemble the vector of initial conditions. Include any
			% variables that were declared by filters as those which need
			% proper initialization.
			xinit = obj.parse_xinit(xinit, varargin{:});
			
			% use a pre-constructed optimizer object for faster
			% evaluation
			[U, status] = obj.optimizer{xinit};
            feasible = ismember(status, [0, 3, 4, 5]);
			if ~feasible
				J = Inf;
				u = NaN(obj.nu, 1);
				U = NaN(size(U));
			elseif isempty(U)
				error('Ooops, something went wrong. Please report to mpt@control.ee.ethz.ch');
			else
				J = U(1);
				u = U(2:obj.nu+1);
			end

			if nargout==3
				% also return the open-loop profiles
				%
				% TODO: also return the 'd' and 'z' variables
				openloop.cost = J;
				U = U(2:end);
				openloop.U = reshape(U(1:obj.nu*obj.N), [obj.nu obj.N]);
				U = U(obj.nu*obj.N+1:end);
				openloop.X = reshape(U(1:obj.nx*(obj.N+1)), [obj.nx, obj.N+1]);
				U = U(obj.nx*(obj.N+1)+1:end);
				openloop.Y = reshape(U(1:obj.model.ny*obj.N), [obj.model.ny, obj.N]);
			end
		end
