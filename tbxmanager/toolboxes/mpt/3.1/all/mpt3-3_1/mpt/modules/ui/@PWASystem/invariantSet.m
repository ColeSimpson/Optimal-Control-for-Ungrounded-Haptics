		function [X, dyn] = invariantSet(obj, varargin)
%
%  INVARIANTSET: Computation of invariant sets for PWA systems 
%  ============================================================
%  
%  
%  SYNTAX
%  ------
%     
%      S = system.invariantSet()
%      S = system.invariantSet('X', X, 'U', U)
%    
%  
%  DESCRIPTION
%  -----------
%     For an autonomous PWA system x^+ = f_PWA(x)  this function computes the set
%  of states for which recursive satisfaction of state constraints can be shown.
%    The set is computed by running the set recursion 
%                                                          
%                          S    = { x  |  f WA(x) in S  }, 
%                           k+1            P          k    
%     initialized by S_0 = X  and terminating once S_k+1=S_k. If X  is not
%  provided, X = { x  |  x_min <= x <= x_max }  is assumed.
%    For a PWA system x^+ = f_PWA(x, u), which is subject to polyhedral state
%  constraints x in X  and input constraints u in U  this function calculates the
%  maximal control-invariant set 
%           C = {x  |  exists u(k) in U, s.t.  x(k) in X,  forall k >= 0}. 
%     Note that this function requires that state constraints defined in
%  system.x.min and system.x.max (see " help SystemSignal").
%  
%  INPUT
%  -----
%     
%        
%          X             Polyhedron defining state constraints    
%                        (optional)                               
%                        Class: polyhedron                        
%          U             Polyhedron defining input constraints    
%                        (optional)                               
%                        Class: polyhedron                        
%          maxIterations Maximal number of iterations (optional)  
%                        Class: double                            
%                          
%  
%  
%  OUTPUT
%  ------
%     
%        
%          S Invariant set                            
%            Class: Polyhedron                        
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
 
 
			% Computes invariant set of the system
			
			global MPTOPTIONS
			if isempty(MPTOPTIONS)
				MPTOPTIONS = mptopt;
			end
			options = MPTOPTIONS.modules.ui.invariantSet;
			ip = inputParser;
			ip.KeepUnmatched = false;
            ip.addParamValue('maxIterations', ...
				options.maxIterations, @isscalar);
            ip.addParamValue('X', ...
				obj.domainx, ...
				@validate_polyhedron);
            ip.addParamValue('U', ...
				obj.u.boundsToPolyhedron(), ...
				@validate_polyhedron);
			ip.addParamValue('merge', true, @islogical);
			ip.parse(varargin{:});
			options = ip.Results;

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

			Xo = options.X;
			U = options.U;
			converged = false;
			for i = 1:options.maxIterations
				fprintf('Iteration %d...\n', i);
				[X, ~, dyn] = obj.reachableSet('X', Xo, 'U', U, ...
					'direction', 'backward', ...
					'merge', true);
				X = X.intersect(obj.x.boundsToPolyhedron);
				if all(X.isEmptySet()) || X==Xo
					converged = true;
					break
				else
					Xo = X;
				end
			end
			if ~converged
				warning('Computation finished without convergence.');
			end
		end
