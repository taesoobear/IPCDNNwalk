/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mrdplot.h"

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

MRDPLOT_DATA *
malloc_mrdplot_data( int n_channels, int n_points )
{
  MRDPLOT_DATA *d;

  d = (MRDPLOT_DATA *) malloc( sizeof( MRDPLOT_DATA ) );
  if ( d == NULL )
    {
      fprintf( stderr, "Couldn't allocate MRDPLOT_DATA.\n" );
      exit( -1 );
    }
  d->filename = NULL;
  d->n_channels = n_channels;
  d->n_points = n_points;
  d->total_n_numbers = n_channels*n_points;
  d->frequency = 0;
  if ( n_channels > 0 )
    {
      d->names = (char **) malloc( d->n_channels*sizeof(char *) );
      d->units = (char **) malloc( d->n_channels*sizeof(char *) );
      if ( d->names == NULL || d->units == NULL )
	{
	  fprintf( stderr,
   "malloc_mrdplot_data: Can't allocate memory for names or units.\n" );
	  exit( -1 );
	}
    }
  else
    {
      d->names = NULL;
      d->units = NULL;
    }
  if ( d->total_n_numbers > 0 )
    {
      d->data = (float *) malloc( d->total_n_numbers*sizeof( float ) );
      if ( d->data == NULL )
	{
	  fprintf( stderr,
	   "malloc_mrdplot_data: Couldn't allocate memory of size %d\n", 
		   d->total_n_numbers );
	  exit( -1 );
	}
    }
  else
    d->data = NULL;
  return d;
}

/*****************************************************************************/

MRDPLOT_DATA *
read_mrdplot( const char *filename )
{
  FILE *stream;
  int total_n_numbers, n_channels, n_points;
  float frequency;
  MRDPLOT_DATA *d;
  int i;
  char buffer1[1000];
  char buffer2[1000];
  char *p;
  int n_bytes;

  stream = fopen( filename, "rb" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Couldn't open %s file for read.\n", 
	       filename );
      exit( -1 );
    }

  if ( fscanf( stream, "%d%d%d%f",
	       &total_n_numbers, 
	       &n_channels, 
	       &n_points, 
	       &frequency ) != 4 )
    {
      fprintf( stderr, "Header error reading %s\n", filename );
      exit( -1 );
    }

  d = malloc_mrdplot_data( n_channels, n_points );
  d->filename = strdup(filename);
  d->frequency = frequency;

  printf(
 "%d points, %d channels in sample, %d numbers total, %g samples/second.\n",
 d->n_points, d->n_channels, 
 d->total_n_numbers, d->frequency );
  

  for( i = 0; i < d->n_channels; i++ )
    {
      fscanf( stream, "%s%s", buffer1, buffer2 );
      d->names[i] = strdup( buffer1 );
      d->units[i] = strdup( buffer2 );
      printf( "%d: %s %s\n", i, d->names[i], d->units[i] );
    }
  fscanf( stream, "%c%c%c", buffer1, buffer1, buffer1 );
  
  /* SGI version */
  
  //fread( d->data, d->n_channels*sizeof( float ), d->n_points, stream );
  
  /* Linux version */
  p = (char *) (d->data);
  n_bytes = d->total_n_numbers*4;
  for( i = 0; i < n_bytes; i += 4 )
    {
      fread( &(p[i+3]), 1, 1, stream );
      fread( &(p[i+2]), 1, 1, stream );
      fread( &(p[i+1]), 1, 1, stream );
      fread( &(p[i+0]), 1, 1, stream );
    }

  fclose( stream );
  return d;
}

/*****************************************************************************/

int
find_channel( char *name, MRDPLOT_DATA *d )
{
  int i;

  for ( i = 0; i < d->n_channels; i++ )
    {
      if ( strcmp( name, d->names[i] ) == 0 )
	{
	  printf( "Found %s at %d\n", name, i );
	  return i;
	}
    }
  printf( "Didn't find %s\n", name );
  return -1;
}

/*****************************************************************************/
/*****************************************************************************/

static void fwrite_reversed( char *p, int i1, int i2, FILE *stream )
{
  int total, i;

  total = i1*i2;

  for( i = 0; i < total; i += 4 )
    {
      fwrite( &(p[i+3]), 1, 1, stream );
      fwrite( &(p[i+2]), 1, 1, stream );
      fwrite( &(p[i+1]), 1, 1, stream );
      fwrite( &(p[i+0]), 1, 1, stream );
    }
}

/*****************************************************************************/

void
write_mrdplot_file( MRDPLOT_DATA *d )
{
  FILE *stream;
  int i;

#ifdef _WIN32
  stream = fopen( d->filename, "wb" ); // windows version
#else
  stream = fopen( d->filename, "w" ); // linux version
#endif

  if ( stream == NULL )
    {
      fprintf( stderr, "Couldn't open %s file for write.\n", d->filename );
      exit( -1 );
    }
#ifdef _WIN32
    fprintf( stream, "%d %d %d %f",
	   d->total_n_numbers, d->n_channels, d->n_points, d->frequency );
	fprintf( stream, "%c", 0x0d);
#else
  fprintf( stream, "%d %d %d %f\n",
	   d->total_n_numbers, d->n_channels, d->n_points, d->frequency );
#endif
  for( i = 0; i < d->n_channels; i++ )
    {
#ifdef _WIN32
		fprintf( stream, "%s %s", d->names[i], d->units[i] );
		fprintf( stream, "%c", 0x0d);
#else
      fprintf( stream, "%s %s\n", d->names[i], d->units[i] );
#endif
    }
#ifdef _WIN32
  fprintf( stream, "%c%c", 0x0d, 0x0d );
#else
  fprintf( stream, "\n\n" );
#endif
  for( i = 0; i < d->n_points; i++ )
    {
      /* SGI version
      fwrite( &(data[i*N_CHANNELS]),
	     N_CHANNELS*sizeof( float ), 1, stream );
	     */
      /* Linux version */
      fwrite_reversed( (char *) (&(d->data[i*d->n_channels])),
	     d->n_channels*sizeof( float ), 1, stream );

    }

  fclose( stream );
}

/*****************************************************************************/

char generated_file_name[10000];

char *
generate_file_name()
{
  FILE *stream;
  int file_number;
  

  stream = fopen( "last_data", "r" );
  if ( stream == NULL )
    file_number = 0;
  else
    {
      fscanf( stream, "%d", &file_number );
      fclose( stream );
    }
  
  sprintf( generated_file_name, "d%05d", file_number );

  file_number++;

  stream = fopen( "last_data", "w" );
  if ( stream == NULL )
    return generated_file_name;
  fprintf( stream, "%d\n", file_number );
  fclose( stream );
  return strdup(generated_file_name);
}

/*****************************************************************************/

char *last_data()
{
  FILE *stream;
  int file_number;

  stream = fopen( "last_data", "r" );
  if ( stream == NULL )
    return strdup( "d00000" );

  fscanf( stream, "%d", &file_number );
  sprintf( generated_file_name, "d%05d", file_number - 1 );
  fclose( stream );
  return strdup( generated_file_name );
}

void free_mrdplot( MRDPLOT_DATA *data ) {
  int i;

  if( data == NULL ) {
    fprintf( stderr, "free_mrdplot: NULL pointer\n" );
    return;
  }

  free( data->filename );
  free( data->data );
  
  for( i=0; i<data->n_channels; i++ ) {
    if( data->names[i] != NULL ) {
      free( data->names[i] );
    }
    if( data->units != NULL ) {
      free( data->units[i] );
    }
  }

  free( data->names );
  free( data->units );
}

/*****************************************************************************/
/*****************************************************************************/

