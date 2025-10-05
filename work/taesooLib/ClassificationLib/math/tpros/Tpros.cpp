#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"


/* ===============================================================

   PROGRAM : Tpros.c

   AUTHOR  : Mark N. Gibbs
                sub-author David J.C. MacKay from 22/11/97

   ===============================================================
   
   This program uses the data to find the most probable
   hyperparameters for a Gaussian process.

   Version Notes 
   =============

Version 1.0 

   4/9/96   This version does not contain the DIAG routines for
            initial optimisation of the noise model. This is 
	    because these routines are still of questionable
	    worth.

	    Installation of new hyp[] and w[] reference scheme
	    to avoid tedious messing around with flags.

   11/9/96  I have re-instated the verbose option for macopt
            because I need it to assess convergence etc.

   9/11/96  I have changed the format of the spec file to deal
            with some new stuff and also because I'm tired of
	    having to write out HUGE command lines.

   9/11/96  I have recombined the training and generation bits
            due to the new sophisication of the specfile.

Version 2.0

   20/11/96 Super excellent new specfile and command line 
            features.

   22/11/96 Installed new periodic covariance function.

   28/11/96 Installed Gamma and Inverse Gamma priors on length
            scales, noise and theta1

   15/12/96 Fixed up CG routine so it wouldn't work with input 
            dependent noise level. Also modified CG routine so that
	    it doesn't require big matrices to be set up.

   09/01/97 Installed log predictive error calculations

Version 2.1

   29/01/97 Added in new hyperparameters for linear term in the
            covariance function

Version 2.2

   05/02/97 Installing multiple Gaussian covariance functions

Version 2.3

   06/02/97 Installed new form of Gamma priors - NOT backwards
            compatible with the old ones !!

   07/02/97 Updated the priors again so that you can have one prior
            on each length scale. Still backwards compatible with
	    previous days alterations

Version 4.0

   22/02/97 This new version 4.0 does not contain any of the polynomial
            routines for the length scales. Modified output facilities
	    which put in comment lines for the hyp file and automatically
	    ignoring them on reading them in.

Version 4.1

   12/03/97 Major changes in the form of the spec files. We split
            the files into sections such that we can have an
	    intelligent users version and a thick prat version.
	    This is hopefully still backward compatible. Also
	    modifying the priors (again) to allow different 
	    specification of the gamma distributions.
	    
Version 4.2

   25/03/97 Major changes in the representation of the length scales
            to implement variable length scales. Internally the length
	    scales are now stored as exponential coefficients. However
	    the I/O stuff converts them back. 
	    This means that even if we are dealing with several 
	    polynomial coefficients for the length scales, they are 
	    ALL converted back into length scale units even though this
	    doesn't mean a great deal. This was done primarily so that
	    compatibility could be maintained with previous versions.

Version 4.3

   26/03/97 Added variable mean. Check out the same problems that I had
            with the Pi variable mean. Also added gaussian prior over the
	    mean and all the necessary I/O jazz. Fixed a few more bugs
	    in the I/O stuff for the priors.

   02/04/97 Added calculation of Hessian for the length scales. This is
            not documented or set up fully. Only accessible through the
	    specfile using 'INT_ls_error_(yes/no)'

   02/04/97 Modified the priors AGAIN. See notes below

   09/05/97 Added in feature which allows noise levels (if previously known
            at every data point) to be loaded in from target file and used. 
	    Obviously you shouldn't try to optimize these as well !!!

Vesion 5.0

   23/05/97 Major revisions to CG routines. Corrected several errors in the
            trace routines as well as fixing up the double-CG algorithm to
	    monitor bounds. No tridiag stuff as it's unreliable.

   23/05/97 I have removed the Legendre polynomial stuff for the variable 
            noise level as this is just not a good way to do it. For the time
	    being I shall use radial basis functions which will be read in 
	    from a file in a similar way to the length scales case. Any one
	    attempting to use Legendre stuff will be warned that the feature
	    has been canned.

Version 5.1

   27/05/97 Have installed tridiagonal CG routine which works. It is slightly
            less stable than the double-CG method but faster. It has a flag
	    associated with it so that you can switch between the two.

   30/06/97 Have disabled the motion of the centres of the RBF stuff for the 
            noise model as it doesn't work too well.

   01/07/97 Have installed a neural network to model the variation of the noise.
            Ironic, huh?

   23/07/97 Modifications to the mac_inter_C and mac_inter_dif routines to try
            to speed them up. I reckon that they are the principle contributors
	    to program slowness and so need sorting out. However we stop short
	    of tabulating the (ki-kj)^2 terms as this would create a very big
	    matrix that, for large problems, would cause trouble.

   30/07/97 Changed the spatially varying length scales stuff so that a constant
            time is included. This constant has a different prior to all the rest.

Version 5.2

   08/08/97 Modified spatial varying lengths scales. Using a new covariance function
            with constant variance (i.e. C(x,x) is not a function of x).

   22/11/97 Added command line option to disable interpolation.
            Made usage more grammatical and more ugly.
	    Changed epsilon to  0.001 by default.
	    Added max_include 0/1 vector which indicates whether
	    an input can be adjusted in the optimization.

Version 5.3

Extensive changes to code.
Introduced subroutines to try to remove redundancy in code.
Replaced hack in macopt with a new structure entry allowing
the state to be written each iteration.

Restored CBJ changes from Tpros.c.OLD

o pred_err is the log likelihood, so can range between +/- infinity.
  - the more positive it is, the better. (Not changed)
o I have added another way of entering the two parameters for the
  inverse gamma function which specifies the noise prior. This new way
  is signalled with noiseprior=3. It specifies the a and b parameters
  of the inverse gamma function, as specified on p.25 of your notebook.
  E.g.

  noise_prior_cbja   1.4
  noise_prior_cbjb   2.5
  
  My definition of b is the inverse of what this program uses, and so
  the read-in value has to be inverted. The reason for this modification
  it that the inverse gamma function is well defined even when its mean
  and standard deviation are not. The mean appears to be undefined if
  a < 1, and the standard deviation undefined if a < 2. This modification
  means that we can use any (positive) value of a, giving a greater
  range of shapes of the required function.
  (Note that this option has not been
  implemented as a command line input.) 

       Removed randomization of initial condition for some hyperparameters

version 5.4

       Putting things in main into subroutines instead.
       Making structures.
       Changing flags to named integers.
       Modifying default priors on hyperparameters.
       Changed mean, d.o.f. to a,b conversion in priors

version 6.0

       Writes log of the input search in a file.

version 6.1 (modifications by CBJ)

       Extra column (the N sigma error) written to the INT_output_file
       when error bar calculation is selected.
       Warning given if attempt made to calculate evidence when perform
       interpolation option is off.

version 6.2 DJCM

       Both optimizations can now have their 'style' changed ,
       and their tolerances too. Style = whether to stop when
       the gradient is small (0) or when the latest step is small (1).

version 6.3 DJCM

       Corrected two minor errors: temp1=0.0 initialization, and "nr"
       should be "npoly"  Tue 30/11/99

       

       */
       

/*

Notes on the handling of the priors

length scales 

       For the non-polynomial length scales, a Gamma prior is used.
       This uses the (m,strength) representation.
       For polynomial length scales, a Gaussian prior with zero mean
       is placed on the coefficients. The associated sd of this 
       distribution can be set using the sd option only.

theta_0

       This uses a Gaussian prior with adjustable mean and variance

theta_1

       This uses an Inverse Gamma prior with the (m,dof)
       representation.

theta_2

       This uses an Inverse Gamma prior with the (m,dof)
       representation.

noise

       For the non-polynomial noise model, an Inverse Gamma prior 
       is used. This uses the (m,dof) representation.
       For polynomial noise model, a Gaussian prior with zero mean
       is placed on the coefficients. The associated sd of this 
       distribution can be set using the sd option only.

linear term

       This uses a Gaussian prior on all its coefficients with 
       adjustable mean and variance through the (m,sd) option. ??

*/

#include <stdio.h>   
#include <stdlib.h>   
#include <math.h>
#include "tpros_nrutil.h"
#include "rand2.h"
#include "mynr.h"
#include "macopt.h"
#include "tpros.h"


	

/* ======================== Main Program ======================= */

void _tmain(int argc,char *argv[])
{  
	tpros gp("tpros spec/Tspec");
	//tpros gp(argc, argv);	
	vectorn target;
	matrixn grid;
	tpros::loadGridAndTarget(gp.numTrainingData(), gp.inputDimension(), grid, target, gp.datafile, gp.gridfile);
	gp.learn(grid,target);
	gp.generateOutput();
}


