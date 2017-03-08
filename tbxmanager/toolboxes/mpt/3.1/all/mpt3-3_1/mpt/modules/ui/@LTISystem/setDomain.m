		function obj = setDomain(obj, type, domain)
%
%  SETDOMAIN: Sets the domain of an LTI system 
%  ============================================
%  
%  
%  SYNTAX
%  ------
%     
%      sys.setDomain('x', P)
%      sys.setDomain('u', P)
%      sys.setDomain('xu', P)
%    
%  
%  DESCRIPTION
%  -----------
%     Sets the region of the state-input space where a given LTI system is valid.
%    This function sets the domain property of LTISystem objects to the polyhedron
%  P, provided as the second input.
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
 
 
			% Sets domain of the system
			
			global MPTOPTIONS
			if isempty(MPTOPTIONS)
				MPTOPTIONS = mptopt;
			end
			
			error(nargchk(3, 3, nargin, 'struct'))
			if ~ischar(type)
				error('The first input must be a string.');
			end
			if ~isa(domain, 'Polyhedron')
				error('The second input must be a polyhedron.');
			end
			function check_dimension(domain, required_dimension)
				if domain.Dim ~= required_dimension
					error('The domain must have dimension %d.', required_dimension);
				end
			end
			% this anonymous function constructs an infinity box of
			% dimension 'n', which emulates R^n
			infbox = @(n) Polyhedron('lb', -MPTOPTIONS.infbound*ones(n, 1), 'ub', MPTOPTIONS.infbound*ones(n, 1));
			domain.minHRep();
			switch type
				case 'x',
					% check and extend the domain to x-u space
					check_dimension(domain, obj.nx);
					obj.domain = domain*infbox(obj.nu);
					obj.domainx = domain;
				case 'u'
					% check and extend the domain to x-u space
					check_dimension(domain, obj.nu);
					obj.domain = infbox(obj.nx)*domain;
					
					% TODO: remove the hard-coded 'fourier' method once
					% Polyhedron/projection is reliable
					obj.domainx = obj.domain.projection(1:obj.nx, 'fourier');
					
				case 'xu',
					% x-u domain provided, just check
					check_dimension(domain, obj.nx+obj.nu);
					obj.domain = domain;
					
					% TODO: remove the hard-coded 'fourier' method once
					% Polyhedron/projection is reliable
					obj.domainx = obj.domain.projection(1:obj.nx, 'fourier');
					
				otherwise
					error('Unrecognized option "%s".', type);
			end
		end
