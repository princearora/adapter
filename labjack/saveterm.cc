
#include  "saveterm.hh"



saveterm::saveterm( void )
{
  tcgetattr(0, &termOrig);
  termNew = termOrig;
  termNew.c_lflag &= ~ICANON;
  termNew.c_lflag &= ~ECHO;
  termNew.c_lflag &= ~ISIG;
  termNew.c_cc[VMIN] = 1;
  termNew.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &termNew);
  peek = -1;
}



saveterm::~saveterm( void )
{
  tcsetattr(0,TCSANOW, &termOrig);
} //  unsetTerm()



int saveterm::kbhit( void )
{
  char ch;
  int nread;

  //if(peek != -1) 
  //  return 1;

  termNew.c_cc[VMIN]=0;
  tcsetattr(0, TCSANOW, &termNew);
  nread = read(0,&ch,1);
  termNew.c_cc[VMIN]=1;
  tcsetattr(0, TCSANOW, &termNew);

  if(nread == 1) 
  {
    //peek = ch;
    return ch;
  }

  return 0;
} //  kbhit()
