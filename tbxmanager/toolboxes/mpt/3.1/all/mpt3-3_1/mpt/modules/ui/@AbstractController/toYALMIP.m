		function Y = toYALMIP(obj)
%
%  TOYALMIP: Converts an MPC problem into YALMIP's constraints and objective 
%  ==========================================================================
%  
%  
%  SYNTAX
%  ------
%     
%      yalmipdata = controller.toYALMIP()
%    
%  
%  DESCRIPTION
%  -----------
%     This function convers an MPC optimization problem into YALMIP. The output
%  structure contains following fields: 
%    
%     - constraints: contains constraints of the MPC problem as YALMIP's lmi
%     object. 
%     - objective: scalar variable of type sdpvar which defines the optimization
%     objective. 
%     - variables: structure containing variables of the optimization problem. 
%    This method is MPT3 replacement of MPT2's ownmpcmechanism. In short, toYALMIP
%  allows to modify the MPC problem by adding new constraints and/or by modifying
%  the objective function.
%  
%  OUTPUT
%  ------
%     
%        
%          yalmipdata Structure containing constraints,        
%                     objective, and variables of the MPC      
%                     problem.                                 
%                     Class: struct                            
%                       
%  
%  
%  SEE ALSO
%  --------
%     fromYALMIP
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
 
 
			% Converts the optimization setup into YALMIP's objects
			%
			% NOTE: derived classes which implement control algorithms that
			% do not have a straightforward YALMIP implementation should
			% re-implement this method to throw an error
			
			% make sure we have the prediction horizon available
			error(obj.assert_controllerparams_defined);
			
			Y = struct('constraints', [], 'objective', [], ...
				'variables', [], 'internal', []);
			obj.model.instantiate(obj.N);
			Y.constraints = obj.model.constraints();
			Y.objective = obj.model.objective();
			
			% TODO: automatically include other variables using
			% getComponents
			Y.variables = struct('x', obj.model.getVariables('x'), ...
				'u', obj.model.getVariables('u'), ...
				'y', obj.model.getVariables('y'));
			
			if isa(obj.model, 'PWASystem')
				Y.variables.d = obj.model.getVariables('d');
				
			elseif isa(obj.model, 'MLDSystem')
				Y.variables.d = obj.model.getVariables('d');
				Y.variables.z = obj.model.getVariables('z');
			end
			
			% list of variables which we ask for from the optimizer in the
			% following order:
			%  * cost (scalar)
			%  * inputs (nu*N)
			%  * states (nx*N)
			%  * outputs (ny*N)
			%  * all other model variables
			f = fieldnames(Y.variables);
			% the cost must always be the first variable
			vars = Y.objective;
			main_variables = {'u', 'x', 'y'};
			for i = 1:length(main_variables)
				v = Y.variables.(main_variables{i});
				vars = [vars; v(:)];
			end
			
			% now add all other variables (e.g. 'd' for PWA, 'd','z' for
			% MLD), dive recursively into automatically introduced
			% variables
			%
			% TODO: no need to include all variables since we only need
			% "u", "x", "y" in evaluate()
			for i = 1:length(f)
				if ~ismember(f{i}, main_variables)
					v = Y.variables.(f{i});
					vars = struct2vars(v(:), vars);
				end
			end
			
			% include additional variables introduced by filters
			add = containers.Map;
			keys = fields(Y.variables);
			for i = 1:length(keys)
				add(keys{i}) = obj.model.(keys{i}).applyFilters('getVariables', 'map');
			end
			% always add 'model' filters
			add('model') = obj.model.applyFilters('getVariables', 'map');
			% any variables introduced by filters?
			new_variables = false;
			keys = add.keys;
			for i = 1:length(keys)
				if ~isempty(add(keys{i}))
					new_variables = true;
					break
				end
			end
			
			% initial conditions for the optimization
			init_vars = struct('component', 'x', ...
				'name', 'x.init', ...
				'var', obj.model.x.var(:, 1), ...
				'dims', size(obj.model.x.var(:, 1)));
			
			if new_variables
				% include variables introduced by filters
				[filter_vars, init_vars] = map2struct(add, init_vars);
				Y.variables.filters = filter_vars;
			end
			
			% store format of the initial condition
			xinit_variables = [];
			obj.xinitFormat.names = {};
			obj.xinitFormat.components = {};
			obj.xinitFormat.dims = {};
			for i = 1:length(init_vars)
				xinit_variables = [xinit_variables; init_vars(i).var(:)];
				obj.xinitFormat.names{end+1} = init_vars(i).name;
				obj.xinitFormat.components{end+1} = init_vars(i).component;
				obj.xinitFormat.dims{end+1} = init_vars(i).dims;
			end
			% number of required initial conditions
			obj.xinitFormat.n_xinit = length(xinit_variables);
			% sdpvars of initial conditions
			Y.internal.parameters = xinit_variables;
			% sdpvars of all requested variables
			Y.internal.requested = vars;
			% format of the initial condition
			Y.internal.xinitFormat = obj.xinitFormat;
            Y.internal.sdpsettings = sdpsettings('verbose', 0);

		end
