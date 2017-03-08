		function out = toSystem(obj)
%
%  TOSYSTEM: Converts the closed-loop object to an autonomous dynamical system 
%  ============================================================================
%  
%  
%  SYNTAX
%  ------
%     
%      sys = loop.toSystem()
%    
%  
%  DESCRIPTION
%  -----------
%     This function converts the closed-loop object to an autonomous dynamical
%  system by using the state feedback u = f(x), where f(x)  represents the explicit
%  form of the controller.
%    If the system is an LTI system (i.e., x^+ = A x + B u), and the controller is
%  defined over a single region (i.e., f(x) = Kx+k), this function produces an
%  instance of the LTISystem class with the dynamic x^+ = (A+BK) x + Bk.
%    If the system is PWA, or if the controller is defined over multiple regions,
%  the toSystem() method produces an autonomous system as an instance of the
%  PWASystemclass. The number of dynamics is equal to the number of regions, over
%  which the controller is defined.
%    In both cases the returned objects represent autonomous systems, i.e. their B
%  and D matrices are empty.
%    Note: this function requires that the closed-loop's controller is available in
%  its explicit form.
%  
%  OUTPUT
%  ------
%     
%        
%          sys Autonomous dynamical system representing 
%              the closed-loop system.                  
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
 
 
			% Converts a closed loop system into a LTI/PWA system
			
			if ~obj.controller.isExplicit()
				error('Only explicit controllers supported.');
			end
			
			if numel(obj.controller.optimizer)>1
				error('Overlapping partitions not supported.');
			end
			
			if obj.controller.optimizer.Dim ~= obj.controller.model.nx
				error('Tracking controllers not supported.');
            end

            if isa(obj.system, 'ULTISystem') && length(obj.controller.optimizer.Set)==1
                % Uncertain LTI system + linear controller
                
                feedback = obj.controller.optimizer.Set.getFunction('primal');
                F = feedback.F(1:obj.system.nu, :);
                g = feedback.g(1:obj.system.nu);
                assert(nnz(g)==0, 'Only linear controllers are supported.');
                unc = obj.system.cellifyMatrices();

                % 1) domain of the closed-loop system
                A = []; b = [];
                % umin <= F*x+g <= umax
                A = [A; F; -F];
                b = [b; obj.system.u.max-g; -obj.system.u.min+g];
                % xmin <= x <= xmax
                A = [A; eye(obj.system.nx); -eye(obj.system.nx)];
                b = [b; obj.system.x.max; -obj.system.x.min];
                % ymin <= (C*x+D*u) <= ymax
                for ic = 1:numel(unc.C)
                    for id = 1:numel(unc.D)
                        A = [A; ...
                            (unc.C{ic}+unc.D{id}*F); ...
                            -(unc.C{ic}+unc.D{id}*F)];
                        b = [b; obj.system.y.max-unc.D{id}*g; ...
                            unc.D{id}*g-obj.system.y.min];
                    end
                end
                D = Polyhedron('A', A, 'b', sanitize_inf(b));
                D = D.intersect(obj.system.domainx);

                % 2) uncertain dynamics of the closed-loop system
                nA = numel(unc.A);
                nB = numel(unc.B);
                An = cell(1, nA*nB);
                i = 0;
                for ia = 1:nA
                    for ib = 1:nB
                        i = i + 1;
                        An{i} = unc.A{ia}+unc.B{ib}*F;
                    end
                end
                out = ULTISystem('A', An, 'C', obj.system.C, ...
                    'D', obj.system.D, 'domain', D);
                out.d = obj.system.d.copy();
                
            elseif isa(obj.system, 'ULTISystem')
                error('Uncertain LTI systems with PWA controllers not yet supported.');
                
            elseif isa(obj.system, 'LTISystem') && length(obj.controller.optimizer.Set)==1
				% LTI system + linear controller = LTI system with
				% domain restricted to the set where the controller
				% satisfies constraints (not necessarily recursively,
				% though)
				
				feedback = obj.controller.optimizer.Set.getFunction('primal');
				F = feedback.F(1:obj.system.nu, :);
				g = feedback.g(1:obj.system.nu);
				
				% 1) domain of the closed-loop system
				A = []; b = [];
				
				% umin <= F*x+g <= umax
				A = [A; F; -F];
				b = [b; obj.system.u.max-g; -obj.system.u.min+g];
				
				% xmin <= x <= xmax
				A = [A; eye(obj.system.nx); -eye(obj.system.nx)];
				b = [b; obj.system.x.max; -obj.system.x.min];
				
				% ymin <= y <= ymax
				A = [A; (obj.system.C+obj.system.D*F); -(obj.system.C+obj.system.D*F)];
				b = [b; obj.system.y.max-obj.system.D*g; obj.system.D*g-obj.system.y.min];
				
				D = Polyhedron('A', A, 'b', sanitize_inf(b));
				D = D.intersect(obj.system.domainx);
				
				% 2) dynamics of the closed-loop system
				An = obj.system.A + obj.system.B*F;
				Bn = zeros(obj.system.nx, 0);
				Cn = obj.system.C + obj.system.D*F;
				Dn = zeros(obj.system.ny, 0);
				fn = obj.system.B*g;
				gn = obj.system.D*g;
				
				% 3) construct the LTI system
				out = LTISystem('A', An, 'B', Bn, 'C', Cn, 'D', Dn, ...
					'f', fn, 'g', gn, 'domain', D);
				
			elseif isa(obj.system, 'LTISystem') || isa(obj.system, 'PWASystem')
				% LTI or PWA system + PWA controller = PWA system
				
				R = obj.controller.feedback.Set;
				% TODO: deal with guardU
				Dom = obj.system.domainx;
				
				A = {}; B = {}; C = {}; D = {}; f = {}; g = {};
				Rn = [];
				for i = 1:length(R)
					% extract parameters of the affine control law u=F*x+G
					for j = 1:length(Dom)
						P = R(i).intersect(Dom(j));
						if P.isFullDim
							F = R(i).Func{1}.F(1:obj.system.nu, :);
							G = R(i).Func{1}.g(1:obj.system.nu);
							A{end+1} = obj.system.A + obj.system.B*F;
							B{end+1} = zeros(obj.system.nx, 0);
							f{end+1} = obj.system.B*G;
							C{end+1} = obj.system.C + obj.system.D*F;
							D{end+1} = zeros(obj.system.nu, 0);
							g{end+1} = obj.system.D*G;
							Rn = [Rn, P];
						end
					end
				end
				out = PWASystem('A', A, 'B', B, 'C', C, 'D', D, ...
					'f', f, 'g', g, 'domain', Rn, 'Ts', obj.system.Ts);
			else
				error('Unsupported system.');
			end
			out.x = obj.system.x.copy();
			out.u = SystemSignal;
			out.y = obj.system.y.copy();
		end
