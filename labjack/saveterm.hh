/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

class: saveterm for C++ (g++) Linux

Started by Carl Friis-Hansen (c) 2006 (carl.friis-hansen@carl-fh.com).
This software is with GPL (use it as you feel like).

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

This class saved the current terminal setting when constructed and
restores the original settings when destoyed.
In addition it provides a kbhit() facility that can be used to scan
for keypress and return the pressed character.

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Change log:

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



#ifndef _SAVETERM_HH_
#define _SAVETERM_HH_

#include <unistd.h>
#include  <termios.h>


class saveterm
{

  protected:

  struct termios termNew;
  struct termios termOrig;
  int    peek;



  public:
          saveterm( void );

          ~saveterm( void );

  int     kbhit( void );

};



#endif
