
#include  "termbar.hh"

termbar::termbar( void )
{
}

termbar::~termbar( void )
{
}



const char*   termbar::bar( double val, double min, double max, int len )
{
  int         nBar, n;

  if( len > 79 )  len = 79;
  if( len < 2 )   len = 2;
  if( min >= max ) {
    for( n=0; n<len; n++ ) { //  Indicate error
      termBarStr[n] = '?';
    }
    termBarStr[n] = '\0';
    return  termBarStr;
  }
  if( val > max ) {
    for( n=0; n<len; n++ ) { //  Indicate over
      termBarStr[n] = '>';
    }
    termBarStr[n] = '\0';
    return  termBarStr;
  }
  if( val < min ) {
    for( n=0; n<len; n++ ) { //  Indicate under
      termBarStr[n] = '<';
    }
    termBarStr[n] = '\0';
    return  termBarStr;
  }
  nBar = (int) ((val-min)/(max-min)*len);
  for( n=0; n<nBar; n++ ) { //  The filled bar part
    termBarStr[n] = '*';
  }
  for( n=nBar; n<len; n++ ) { //  The non-filled bar part
    termBarStr[n] = '.';
  }
  termBarStr[n] = '\0';
  return  termBarStr;
}
