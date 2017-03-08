		function [S, SN, dyn, dynN] = reachableSet(obj, varargin)
%
%  REACHABLESET: Computes forward or backwards reachable sets 
%  ===========================================================
%  
%  
%  SYNTAX
%  ------
%     
%      [S, SN] = system.reachableSet('X', X, 'U', U, 'N', N, 'direction',
%  ['forward'|'backwards'])
%    
%  
%  DESCRIPTION
%  -----------
%     S = system.reachableSet('X', X, 'U', U, 'direction', 'forward') computes the
%  forward reachable set for a PWA system, i.e., 
%                                                             
%                       S = { f WA(x, u)  |  x in X, u in U } 
%                              P                              
%     S = system.reachableSet('X', X, 'U', U, 'direction', 'backward') computes the
%  backward reachable set, i.e., 
%                                                             
%                       S = { x  |  f WA(x, u) in X, u in U } 
%                                    P                        
%     If X and/or U are not given, the constraints are extracted from state and
%  input constraints. If directionis not specified, direction='forward' is assumed.
%  
%    This function supports autonomous systems as well.
%  
%  INPUT
%  -----
%     
%        
%          X         Polyhedron defining state constraints    
%                    (optional)                               
%                    Class: polyhedron                        
%          U         Polyhedron defining input constraints    
%                    (optional)                               
%                    Class: polyhedron                        
%          direction Flag to switch between forward and       
%                    backwards reachability.                  
%                    Class: char                              
%          N         Number of steps (defaults to 1).         
%                    Class: double                            
%                      
%  
%  
%  OUTPUT
%  ------
%     
%        
%          S  Set of states reachable in N  steps.     
%             Class: Polyhedron                        
%          SN Set of states reachable at each step.    
%             Class: cell                              
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
 
 
			% Computes the forward/backwards reachable N-step set
			ip = inputParser;
			ip.KeepUnmatched = false;
			ip.addParamValue('direction', 'forward', @ischar);
			ip.addParamValue('N', 1, @isnumeric);
            ip.addParamValue('X', ...
				[], ...
				@validate_polyhedron);
            ip.addParamValue('U', ...
				[], ...
				@validate_polyhedron);
			ip.addParamValue('merge', true, @islogical);
			ip.parse(varargin{:});
			options = ip.Results;

            if isempty(options.X)
                options.X = obj.x.boundsToPolyhedron();
            end
            if isempty(options.U)
                options.U = obj.u.boundsToPolyhedron();
            end
            
			if numel(options.U)~=1
				error('Input constraints must be a single polyhedron.');
			end
			if obj.nu>0 && options.U.isEmptySet()
				error('Input constraints must not be empty.');
			end
			if any(arrayfun(@(x) x.Dim~=obj.nx, options.X))
				error('State constraints must be a polyhedron in %dD.', obj.nx);
			end
			if obj.nu>0 && options.U.Dim~=obj.nu
				error('Input constraints must be a polyhedron in %dD.', obj.nu);
			end

			X = options.X*options.U;
			SN = {}; dynN = {};
			for n = 1:options.N
				Xp = []; dyn = [];
				for j = 1:obj.ndyn
					lti = obj.toLTI(j);
					XD = X.projection(1:obj.nx);
					R = lti.reachableSet('N', 1, ...
						'direction', options.direction, ...
						'X', XD, 'U', options.U);
					% is the union of "R" convex?
					if numel(R)>1
						H = PolyUnion(R);
						if H.isConvex
							R = H.convexHull;
						elseif options.merge
							R = PolyUnion(R).merge().Set;
						end
					end
					Xp = [Xp, R];
					dyn = [dyn j*ones(1, numel(R))];
				end
				X = Xp*options.U;
				SN{n} = Xp;
				dynN{n} = dyn;
			end
			S = SN{end};
            if isempty(S)
                S = Polyhedron.emptySet(obj.nx);
            end
			dyn = dynN{end};
		end
