%%
%%	This file is part of qpOASES.
%%
%%	qpOASES -- An Implementation of the Online Active Set Strategy.
%%	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
%%	Christian Kirches et al. All rights reserved.
%%
%%	qpOASES is free software; you can redistribute it and/or
%%	modify it under the terms of the GNU Lesser General Public
%%	License as published by the Free Software Foundation; either
%%	version 2.1 of the License, or (at your option) any later version.
%%
%%	qpOASES is distributed in the hope that it will be useful,
%%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%%	See the GNU Lesser General Public License for more details.
%%
%%	You should have received a copy of the GNU Lesser General Public
%%	License along with qpOASES; if not, write to the Free Software
%%	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%



%%
%%	Filename:  interfaces/octave/make.m
%%	Author:    Hans Joachim Ferreau, Andreas Potschka
%%	Version:   3.0beta
%%	Date:      2007-2012
%%



% consistency check
if ( exist( [pwd, '/make.m'],'file' ) == 0 )
	disp( 'ERROR: Run this make script directly within the directory' );
	disp( '       <qpOASES-inst-dir>/INTERFACES/MATLAB, please.' );
	return;
end


QPOASESPATH = '../../';

IFLAGS  = [ '-I. -I',QPOASESPATH,'include  -I',QPOASESPATH,'include/qpOASES ' ];

CPPFLAGS  = [ IFLAGS, '-D__cpluplus -D__MATLAB__ -DLINUX', ' ' ]; %% -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__

QPOASES_OBJECTS =	[	QPOASESPATH, 'src/SQProblem.cpp ',...
						QPOASESPATH, 'src/QProblem.cpp ',...
						QPOASESPATH, 'src/QProblemB.cpp ',...
						QPOASESPATH, 'src/Bounds.cpp ',...
						QPOASESPATH, 'src/Constraints.cpp ',...
						QPOASESPATH, 'src/SubjectTo.cpp ',...
						QPOASESPATH, 'src/Indexlist.cpp ',...
						QPOASESPATH, 'src/Flipper.cpp ',...
						QPOASESPATH, 'src/Utils.cpp ',...
						QPOASESPATH, 'src/Options.cpp ',...
						QPOASESPATH, 'src/Matrices.cpp ',...
						QPOASESPATH, 'src/BLASReplacement.cpp ',...
						QPOASESPATH, 'src/LAPACKReplacement.cpp ',...
						QPOASESPATH, 'src/MessageHandling.cpp ', ' ' ];


DEBUGFLAGS = ' -g ';
%DEBUGFLAGS = ' -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

NAME = 'qpOASES';
eval( [ 'mkoctfile --mex --output ', NAME, '.', mexext(), ' ', CPPFLAGS, DEBUGFLAGS, NAME,'.cpp ',QPOASES_OBJECTS ] )
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

NAME = 'qpOASES_sequence';
eval( [ 'mkoctfile --mex --output ', NAME, '.', mexext(), ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

NAME = 'qpOASES_sequenceSB';
eval( [ 'mkoctfile --mex --output ', NAME, '.', mexext(), ' ', CPPFLAGS, DEBUGFLAGS, [NAME,'.cpp ',QPOASES_OBJECTS] ] );
disp( [ NAME, '.', eval('mexext'), ' successfully created!'] );

path( path,pwd );


clear QPOASESPATH IFLAGS CPPFLAGS QPOASES_OBJECTS DEBUGFLAGS NAME



%%
%%	end of file
%%
