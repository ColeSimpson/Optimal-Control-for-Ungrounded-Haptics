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
%     u = controller.evaluate(x0) evaluates the explicit MPC solution for the
%  initial condition x0 and returns the first element of the optimal sequence of
%  control inputs. If x0 is outside of the controller's domain, u will be NaN.
%     [u, feasible] = controller.evaluate(x0) also returns the boolean flag
%  indicating whether x0 is inside of the controller's domain.
%     [u, feasible, openloop] = controller.evaluate(x0) also returns the open-loop
%  predictions of states, inputs and outputs in openloop.X, openloop.Y, and
%  openloop.Y, respectively. Value of the optimal cost is returned in
%  openloop.cost.
%  
%  INPUT
%  -----
%     
%        
%          x0 Initial state.                           
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
%          feasible True if a feasible control actions       
%                   exists for x0, false otherwise.          
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
			% u = controller.evaluate(x0) evaluates the explicit MPC
			% solution and returns the optimal control input associated to
			% point "x0". If "x0" is outside of the controller's domain,
			% "u" will be NaN.
			%
			% [u, feasible] = controller.evaluate(x0) also returns a
			% feasibility boolean flag.
			%
			% [u, feasible, openloop] = controller.evaluate(x0) also
			% returns the full open-loop optimizer in "openloop.U" and the
			% optimal cost in "openloop.cost". Moreover, "openloop.X" and
			% "openloop.Y" will be set to NaN. This is due to the fact that
			% storing open-loop predictions of states and outputs in the
			% explicit solution would considerably increase its size.
			%
			% u = controller.evaluate(x0, 'x.reference', ref, 'u.prev', u0)
			% includes "ref" and "u0" into the vector of initial
			% conditions.
			% TODO: point location should be a method of PolyUnion
			% make sure the prediction horizon was provided
			error(obj.assert_controllerparams_defined);
			if isempty(obj.nu)
				error('Set ctrl.nu first.');
			end
			
			error(validate_vector(xinit, obj.nx, 'initial state'));
			
			% assemble the vector of initial conditions. Include any
			% variables that were declared by filters as those which need
			% proper initialization.
			xinit = obj.parse_xinit(xinit, varargin{:});
			
			% evaluate the primal optimizer, break ties based on the cost
			% function. guarantees that the output is a single region where
			% the cost is minimal.
			[U, feasible, idx, J] = obj.optimizer.feval(xinit, ...
				'primal', 'tiebreak', 'obj');
			if ~feasible
				J = Inf;
				% index of the optimizer and index of the region from which the
				% control action was extracted
				opt_partition = [];
				opt_region = [];
			elseif numel(obj.optimizer)==1
				opt_partition = 1;
				opt_region = idx;
			else
				% multiple optimizers
				opt_partition = idx(1);
				opt_region = idx(2);
			end
				
			if isempty(J)
				% no tie-breaking was performed, compute cost manually
				%
				% Note: from a long-term sustainibility point of view
				% we should use
				%   J = obj.optimizer.Set(idx).feval(xinit, 'obj');
				% here. but ConvexSet/feval() adds so much unnecessary
				% overhead that we better evaluate the function
				% directly
				J = obj.optimizer(opt_partition).Set(opt_region).Functions('obj').feval(xinit);
			end				
			
			if numel(U)~=obj.nu*obj.N
				% sanity checks for EMPCControllers imported from
				% polyunions
				error('Number of optimizers is inconsistent with "N" and/or "nu".');
			end
			
			u = U(1:obj.nu);
			if nargout==3
				openloop.cost = J;
				openloop.U = reshape(U, [obj.nu obj.N]);
				openloop.X = NaN(obj.nx, obj.N+1);
				openloop.Y = NaN(obj.model.ny, obj.N);
				openloop.partition = opt_partition;
				openloop.region = opt_region;
			end
		end
