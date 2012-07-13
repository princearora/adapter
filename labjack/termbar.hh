/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

class: termbar for C++ (g++) Linux

Started by Carl Friis-Hansen (c) 2006 (carl.friis-hansen@carl-fh.com).
This software is with GPL (use it as you feel like).

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

This class can make something that looks like a progress bar in semi
graphic format that suits any simpel terminal.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Change log:

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



#ifndef _TERMBAR_HH_
#define _TERMBAR_HH_



class termbar
{

  protected:
    char    termBarStr[80];



  public:
            termbar( void );

            ~termbar( void );

    const char*   bar( double val, double min, double max, int len );

};



#endif
