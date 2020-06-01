//
#ifndef GRAHAM_STAND_ALONE
#include "../physicsLib.h"
#endif
// This program uses the Graham scan algorithm to calculate the convex
// hull of a batch of points
//

#include <iostream>
#include <vector>
#include <ctime>
#include <fstream>
#include <algorithm>
#include <string>
#include "graham.h"


std::ostream &operator <<(std::ostream &s, const std::pair<double,double> &point )
{
    s << "("
      << point.first
      << ","
      << point.second
      << ")";
    return s;
}

GrahamScan:: GrahamScan( size_t n, double xmin, double xmax, double ymin, double ymax )
        : N( n )
        , x_range( xmin, xmax )
        , y_range( ymin, ymax )
    {
        //
        // In this constructor I generate the N random points asked for
        // by the caller. Their values are randomly assigned in the x/y
        // ranges specified as arguments
        //
        //srand( static_cast<unsigned int>( 0));
        for ( size_t i = 0 ; i < N ; i++ ) {
            double x = ( rand() % int( x_range.second - x_range.first + 1 ) ) + x_range.first;
            double y = ( rand() % int( y_range.second - y_range.first + 1 ) ) + y_range.first;
            raw_points.push_back( std::make_pair( x, y ) );
        }
    }

GrahamScan::	GrahamScan()
	{
		N=0;
	}
	void GrahamScan::add_point(std::pair<double, double> const& point)
	{
		double thr=0.02;
		if(false) {
			// check if close point exits
			for (size_t i=0;i<raw_points.size(); i++)
			{
				std::pair<double,double>& p=raw_points[i];
				if ( (p.first-point.first)*(p.first-point.first)+
						(p.second-point.second)*(p.second-point.second)
						<thr*thr)
					return;
			}
		}
		raw_points.push_back(point);
		N=raw_points.size();
	}

    //
    // The initial array of points is stored in vectgor raw_points. I first
    // sort it, which gives me the far left and far right points of the hull.
    // These are special values, and they are stored off separately in the left
    // and right members.
    //
    // I then go through the list of raw_points, and one by one determine whether
    // each point is above or below the line formed by the right and left points.
    // If it is above, the point is moved doubleo the upper_partition_points sequence. If it
    // is below, the point is moved doubleo the lower_partition_points sequence. So the output
    // of this routine is the left and right points, and the sorted points that are in the
    // upper and lower partitions.
    //
    void GrahamScan::partition_points()
    {
        //
        // Step one in partitioning the points is to sort the raw data
        //
        std::sort( raw_points.begin(), raw_points.end() );
        //
        // The the far left and far right points, remove them from the
        // sorted sequence and store them in special members
        //
        left = raw_points.front();
        raw_points.erase( raw_points.begin() );
        right = raw_points.back();
        raw_points.pop_back();
        //
        // Now put the remaining points in one of the two output sequences
        //
        for ( size_t i = 0 ; i < raw_points.size() ; i++ )
        {
            double dir = direction( left, right, raw_points[ i ] );
            if ( dir < 0 )
                upper_partition_points.push_back( raw_points[ i ] );
            else
                lower_partition_points.push_back( raw_points[ i ] );
        }
    }
    //
    // Building the hull consists of two procedures: building the lower and
    // then the upper hull. The two procedures are nearly identical - the main
    // difference between the two is the test for convexity. When building the upper
    // hull, our rull is that the middle point must always be *above* the line formed
    // by its two closest neighbors. When building the lower hull, the rule is that point
    // must be *below* its two closest neighbors. We pass this information to the 
    // building routine as the last parameter, which is either -1 or 1.
    //
	//pu
#ifdef STAND_ALONE
    void GrahamScan::build_hull( std::ofstream &f )
    {
        build_half_hull( f, lower_partition_points, lower_hull, 1 );
        build_half_hull( f, upper_partition_points, upper_hull, -1 );
    }
#else 
		void GrahamScan::build_hull()
    {
        build_half_hull(  lower_partition_points, lower_hull, 1 );
        build_half_hull(  upper_partition_points, upper_hull, -1 );
    }
#endif
    //
    // This is the method that builds either the upper or the lower half convex
    // hull. It takes as its input a copy of the input array, which will be the
    // sorted list of points in one of the two halfs. It produces as output a list
    // of the points in the corresponding convex hull.
    //
    // The factor should be 1 for the lower hull, and -1 for the upper hull.
    //
#ifdef STAND_ALONE
    void GrahamScan::build_half_hull( std::ostream &f, 
#else
			void GrahamScan::build_half_hull(
#endif
                          std::vector< std::pair<double,double> > input,
                          std::vector< std::pair<double,double> > &output,
                          double factor )
    {
        //
        // The hull will always start with the left point, and end with the right
        // point. According, we start by adding the left point as the first point
        // in the output sequence, and make sure the right point is the last point 
        // in the input sequence.
		//
        input.push_back( right );
        output.push_back( left );
        //
        // The construction loop runs until the input is exhausted
        //
        while ( input.size() != 0 ) {
            //
            // Repeatedly add the leftmost point to the null, then test to see 
            // if a convexity violation has occured. If it has, fix things up
            // by removing the next-to-last point in the output suqeence until 
            // convexity is restored.
            //
            output.push_back( input.front() );
#ifdef GRAHAM_STAND_ALONE
            plot_hull( f, "adding a new point" );
#endif
            input.erase( input.begin() );
            while ( output.size() >= 3 ) {
                size_t end = output.size() - 1;
                if ( factor * direction( output[ end - 2 ], 
                                         output[ end ], 
                                         output[ end - 1 ] ) <= 0 ) {
                    output.erase( output.begin() + end - 1 );
#ifdef GRAHAM_STAND_ALONE
                    plot_hull( f, "backtracking" );
#endif
                }
                else
                    break;
            }
        }
    }
    //
    // In this program we frequently want to look at three consecutive
    // points, p0, p1, and p2, and determine whether p2 has taken a turn
    // to the left or a turn to the right.
    //
    // We can do this by by translating the points so that p1 is at the origin,
    // then taking the cross product of p0 and p2. The result will be positive,
    // negative, or 0, meaning respectively that p2 has turned right, left, or
    // is on a straight line.
    //
									  double GrahamScan::direction( std::pair<double,double> p0,
                          std::pair<double,double> p1,
                          std::pair<double,double> p2 )
    {
        return ( (p0.first - p1.first ) * (p2.second - p1.second ) )
               - ( (p2.first - p1.first ) * (p0.second - p1.second ) );
    }
    void GrahamScan::print_raw_points()
	{
		log_raw_points(	std::cout );
	}
    void GrahamScan::log_raw_points( std::ostream &f )
    {
        f << "Creating raw points:\n";
        for ( size_t i = 0 ; i < N ; i++ ) 
            f << raw_points[ i ] << " ";
        f << "\n";
    }
    void GrahamScan::log_partitioned_points( std::ostream &f )
    {
        f << "Partitioned set:\n"
          << "Left : " << left << "\n"
          << "Right : " << right << "\n"
          << "Lower partition: ";
        for ( size_t i = 0 ; i < lower_partition_points.size() ; i++ )
            f << lower_partition_points[ i ];
        f << "\n";
        f << "Upper partition: ";
        for ( size_t i = 0 ; i < upper_partition_points.size() ; i++ )
            f << upper_partition_points[ i ];
        f << "\n";
    }
	void GrahamScan::print_hull()
	{
		log_hull(std::cout);
	}
    void GrahamScan::log_hull( std::ostream & f )
    {
        f << "Lower hull: ";
        for ( size_t i = 0 ; i < lower_hull.size() ; i++ )
            f << lower_hull[ i ];
        f << "\n";
        f << "Upper hull: ";
        for ( size_t i = 0 ; i < upper_hull.size() ; i++ )
            f << upper_hull[ i ];
        f << "\n";
        f << "Convex hull: ";
        for ( size_t i = 0 ; i < lower_hull.size() ; i++ )
            f << lower_hull[ i ] << " ";
        for ( std::vector< std::pair<double,double> >::reverse_iterator ii = upper_hull.rbegin() + 1 ;
              ii != upper_hull.rend();
              ii ++ )
            f << *ii << " ";
        f << "\n";
        
    }
#ifndef GRAHAM_STAND_ALONE
	void GrahamScan::get_hull(matrixn& out)
	{
		out.setSize(lower_hull.size()+upper_hull.size()-1,2);
		for (size_t i=0; i<lower_hull.size(); i++)
		{
			out(i,0)=lower_hull[i].first;
			out(i,1)=lower_hull[i].second;
		}

		int c=lower_hull.size();
		for ( std::vector< std::pair<double,double> >::reverse_iterator ii = upper_hull.rbegin() + 1 ;
				ii != upper_hull.rend();
				ii ++ )
		{
			out(c,0)=(*ii).first;
			out(c,1)=(*ii).second;
			c++;
		}
	}
#endif
    void GrahamScan::plot_raw_points( std::ostream &f )
    {
        f << "set xrange ["
          << x_range.first 
          << ":"
          << x_range.second
          << "]\n";
        f << "set yrange ["
          << y_range.first 
          << ":"
          << y_range.second
          << "]\n";
        f << "unset mouse\n";
        f << "set title 'The set of raw points in the set' font 'Arial,12'\n";
        f << "set style line 1 pointtype 7 linecolor rgb 'red'\n";
        f << "set style line 2 pointtype 7 linecolor rgb 'green'\n";
        f << "set style line 3 pointtype 7 linecolor rgb 'black'\n";
        f << "plot '-' ls 1 with points notitle\n";
        for ( size_t i = 0 ; i < N ; i++ )
            f << raw_points[ i ].first << " " << raw_points[ i ].second << "\n";
        f << "e\n";
        f << "pause -1 'Hit OK to move to the next state'\n";
    }
    void GrahamScan::plot_partitioned_points( std::ostream &f )
    {
        f << "set title 'The points partitioned doubleo an upper and lower hull' font 'Arial,12'\n";
        f << "plot '-' ls 1 with points notitle, "
          << "'-' ls 2 with points notitle, "
          << "'-' ls 3 with linespoints notitle\n";
        for ( size_t i = 0 ; i < lower_partition_points.size() ; i++ )
            f << lower_partition_points[ i ].first 
              << " " 
              << lower_partition_points[ i ].second 
              << "\n";
        f << "e\n";
        for ( size_t i = 0 ; i < upper_partition_points.size() ; i++ )
            f << upper_partition_points[ i ].first 
              << " " 
              << upper_partition_points[ i ].second 
              << "\n";
        f << "e\n";
        f << left.first << " " << left.second << "\n";
        f << right.first << " " << right.second << "\n";
        f << "e\n";
        f << "pause -1 'Hit OK to move to the next state'\n";
    }
    void GrahamScan::plot_hull( std::ostream &f, std::string text )
    {
        f << "set title 'The hull in state: "
          << text
          << "' font 'Arial,12'\n";
        f << "plot '-' ls 1 with points notitle, ";
        if ( lower_hull.size() )
            f << "'-' ls 3 with linespoints notitle, ";
        if ( upper_hull.size() )
            f << "'-' ls 3 with linespoints notitle, ";
        f << "'-' ls 2 with points notitle\n";
        for ( size_t i = 0 ; i < lower_partition_points.size() ; i++ )
            f << lower_partition_points[ i ].first 
              << " " 
              << lower_partition_points[ i ].second 
              << "\n";
        f << right.first << " " << right.second << "\n";
        f << "e\n";
        if ( lower_hull.size() ) {
            for ( size_t i = 0 ; i < lower_hull.size() ; i++ )
                f << lower_hull[ i ].first 
                  << " " 
                  << lower_hull[ i ].second 
                  << "\n";
                f << "e\n";
        }
        if ( upper_hull.size() ) {
            for ( std::vector< std::pair<double,double> >::reverse_iterator ii = upper_hull.rbegin();
                  ii != upper_hull.rend();
                  ii++ ) 
                f << ii->first 
                  << " " 
                  << ii->second 
                  << "\n";
            f << "e\n";
        }
        for ( size_t i = 0 ; i < upper_partition_points.size() ; i++ )
            f << upper_partition_points[ i ].first 
              << " " 
              << upper_partition_points[ i ].second 
              << "\n";
        f << "e\n";
        f << "pause -1 'Hit OK to move to the next state'\n";
    }
#ifdef GRAHAM_STAND_ALONE
double main(double argc, char* argv[])
{
    std::ofstream gnuplot_file( "gnuplot.cmd" );
    const int N = 20;
    GrahamScan g( N, 0, 100, 0, 100 );
    g.log_raw_points( std::cout );
    g.plot_raw_points( gnuplot_file );
    g.partition_points();
    g.log_partitioned_points( std::cout );
    g.plot_partitioned_points( gnuplot_file );
    //
    // Okay, time to start building the hull
    //
    g.build_hull( gnuplot_file );
    g.log_hull( std::cout );
    g.plot_hull( gnuplot_file, "complete" );
    return 0;
}
#else

#ifdef USE_LUABIND
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
#include <luabind/out_value_policy.hpp>
#include <luabind/adopt_policy.hpp>
#include <luabind/error.hpp>

using namespace luabind;
void registerGraham(lua_State* L)
{
	module(L,"math")
	[
	class_<std::pair<double, double> >("Point2D")	
	.def(constructor<>())
	.def(constructor<double,double>())
	.def_readwrite("x",&std::pair<double, double>::first) 
	.def_readwrite("y",&std::pair<double, double>::second),
	class_<GrahamScan>("GrahamScan")
	.def(constructor<>())
	.def("add_point", &GrahamScan::add_point)
	.def("partition_points",&GrahamScan::partition_points)
	.def("build_hull",&GrahamScan::build_hull)
	.def("print_raw_points",&GrahamScan::print_raw_points)
	.def("print_hull",&GrahamScan::print_hull)
	.def("get_hull",&GrahamScan::get_hull)
	.def("direction",&GrahamScan::direction)
	];
}
#endif
#endif
