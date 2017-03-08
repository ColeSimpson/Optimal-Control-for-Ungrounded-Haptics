function display(obj)
%
%  DISPLAY: Overload display for AffFunction class. 
%  =================================================
%  
%  
%  SYNTAX
%  ------
%     
%      display(F)
%      F.display
%    
%  
%  DESCRIPTION
%  -----------
%     Default display for AffFunction class.
%  
%  INPUT
%  -----
%     
%        
%          F AffFunction object.                      
%            Class: Function                          
%              
%  
%  
%  SEE ALSO
%  --------
%     Function,  AffFunction,  QuadFunction
%  

%  AUTHOR(s)
%  ---------
%     
%    
%   (c) 2010-2013  Martin Herceg: ETH Zurich
%   mailto:herceg@control.ee.ethz.ch 
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
 
 
if numel(obj)>1
    fprintf('Array of %d Affine Functions\n',numel(obj));
    return;
end

fprintf('Affine Function: R^%d -> R^%d\n',obj.D,obj.R);
%fprintf('y = F*x + g\n');

end
